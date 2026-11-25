use core::sync::atomic::{Ordering, compiler_fence};

use executor::{ExecutorError, PioCommandBlock, PioCommandBlockReader, PioCommandWord};
use rp235x_hal::{
    dma::{CH0, CH1, Channel, SingleChannel},
    pac::{self, PIO0},
    pio::{SM0, Tx},
};

use crate::dma_command_buffer::DmaCommandBuffer;

type StepDirTx = Tx<(PIO0, SM0)>;
const CHANNEL_MASK: u16 = 0b11;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DmaSlot {
    A,
    B,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefillError {
    Source(ExecutorError),
    MissingBlock,
    DeadlineMissed,
    DmaBusError,
}

/// Transfiere bloques a una misma FIFO reutilizando CH0 y CH1.
#[must_use = "llama a stop para recuperar los canales, los búferes y la FIFO"]
pub struct DmaPairTransfer {
    channel_a: Channel<CH0>,
    channel_b: Channel<CH1>,
    buffer_a: DmaCommandBuffer,
    buffer_b: DmaCommandBuffer,
    tx: StepDirTx,
    next_to_refill: DmaSlot,
    final_queued: bool,
    blocks_queued: usize,
}

#[must_use]
pub struct DmaPairResources {
    pub channel_a: Channel<CH0>,
    pub channel_b: Channel<CH1>,
    pub buffer_a: DmaCommandBuffer,
    pub buffer_b: DmaCommandBuffer,
    pub tx: StepDirTx,
    pub completed: bool,
    pub bus_error: bool,
}

impl DmaPairTransfer {
    pub fn start(
        channel_a: Channel<CH0>,
        channel_b: Channel<CH1>,
        buffer_a: DmaCommandBuffer,
        buffer_b: DmaCommandBuffer,
        tx: StepDirTx,
    ) -> Self {
        let end = PioCommandWord::END.bits();
        assert!(!buffer_a.is_empty());
        let single_block = buffer_a.as_slice().last() == Some(&end);
        assert_eq!(single_block, buffer_b.is_empty());
        let final_queued = single_block || buffer_b.as_slice().last() == Some(&end);

        assert!(!channel_busy(&channel_a));
        assert!(!channel_busy(&channel_b));
        clear_channel_completion(channel_a.id());
        clear_channel_completion(channel_b.id());

        // Elegir el propio canal en CHAIN_TO deshabilita el encadenamiento.
        if single_block {
            disable_channel(&channel_b);
            configure_channel(&channel_a, &buffer_a, &tx, channel_a.id());
        } else {
            configure_channel(&channel_b, &buffer_b, &tx, channel_b.id());
            configure_channel(&channel_a, &buffer_a, &tx, channel_b.id());
        }

        compiler_fence(Ordering::SeqCst);
        cortex_m::asm::dsb();

        // SAFETY: es la dirección de la FIFO TX que posee esta transferencia.
        // CH0 está configurado y ambos búferes estáticos siguen siendo exclusivos.
        channel_a
            .ch()
            .ch_al2_write_addr_trig()
            .write(|w| unsafe { w.bits(tx.fifo_address() as u32) });

        Self {
            channel_a,
            channel_b,
            buffer_a,
            buffer_b,
            tx,
            next_to_refill: DmaSlot::A,
            final_queued,
            blocks_queued: if single_block { 1 } else { 2 },
        }
    }

    fn channel_a_busy(&self) -> bool {
        channel_busy(&self.channel_a)
    }

    fn channel_b_busy(&self) -> bool {
        channel_busy(&self.channel_b)
    }

    pub fn has_bus_error(&self) -> bool {
        channel_has_bus_error(&self.channel_a) || channel_has_bus_error(&self.channel_b)
    }

    pub fn blocks_queued(&self) -> usize {
        self.blocks_queued
    }

    pub fn final_queued(&self) -> bool {
        self.final_queued
    }

    /// Indica que todos los bloques ya se transfirieron a la FIFO.
    pub fn is_done(&self) -> bool {
        self.final_queued
            && channel_completed(self.channel_a.id())
            && (self.blocks_queued == 1 || channel_completed(self.channel_b.id()))
            && !self.channel_a_busy()
            && !self.channel_b_busy()
            && !self.has_bus_error()
    }

    /// Publica el siguiente bloque cuando DMA libera su búfer.
    /// Ante un error, detener la ejecución y recuperar los recursos con `stop`.
    pub fn refill_next<I>(
        &mut self,
        reader: &mut PioCommandBlockReader<I>,
    ) -> Result<Option<PioCommandBlock>, RefillError>
    where
        I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
    {
        if self.has_bus_error() {
            return Err(RefillError::DmaBusError);
        }
        if self.final_queued {
            return Ok(None);
        }

        let block = match self.next_to_refill {
            DmaSlot::A => refill_channel(
                &self.channel_a,
                &self.channel_b,
                &mut self.buffer_a,
                &self.tx,
                reader,
            )?,
            DmaSlot::B => refill_channel(
                &self.channel_b,
                &self.channel_a,
                &mut self.buffer_b,
                &self.tx,
                reader,
            )?,
        };
        if let Some(block) = block {
            self.blocks_queued += 1;
            self.final_queued = block.is_final;
            self.next_to_refill = match self.next_to_refill {
                DmaSlot::A => DmaSlot::B,
                DmaSlot::B => DmaSlot::A,
            };
        }
        Ok(block)
    }

    /// Recupera los recursos cuando ambos canales se han detenido.
    /// Al interrumpir un movimiento, detener primero la máquina PIO.
    pub fn stop(self) -> DmaPairResources {
        let completed = self.is_done();

        // RP2350-E5 exige deshabilitar ambos canales antes de solicitar el aborto.
        disable_channel(&self.channel_a);
        disable_channel(&self.channel_b);
        cortex_m::asm::dsb();

        // SAFETY: las escrituras de aborto y reconocimiento afectan solo a
        // CH0 y CH1, que son propiedad exclusiva de esta transferencia.
        let dma = unsafe { &*pac::DMA::ptr() };
        if !completed {
            dma.chan_abort()
                .write(|w| unsafe { w.chan_abort().bits(CHANNEL_MASK) });
            while dma.chan_abort().read().bits() != 0 {
                core::hint::spin_loop();
            }
        }
        while self.channel_a_busy() || self.channel_b_busy() {
            core::hint::spin_loop();
        }

        let bus_error = self.has_bus_error();
        if bus_error {
            clear_channel_errors(&self.channel_a);
            clear_channel_errors(&self.channel_b);
        }
        dma.intr()
            .write(|w| unsafe { w.bits(u32::from(CHANNEL_MASK)) });
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        DmaPairResources {
            channel_a: self.channel_a,
            channel_b: self.channel_b,
            buffer_a: self.buffer_a,
            buffer_b: self.buffer_b,
            tx: self.tx,
            completed,
            bus_error,
        }
    }
}

fn channel_completed(id: u8) -> bool {
    // SAFETY: leer INTR no modifica los indicadores DMA.
    let dma = unsafe { &*pac::DMA::ptr() };
    dma.intr().read().bits() & (1u32 << id) != 0
}

fn clear_channel_completion(id: u8) {
    // SAFETY: solo se reconoce el indicador del canal poseído por el par.
    let dma = unsafe { &*pac::DMA::ptr() };
    dma.intr().write(|w| unsafe { w.bits(1u32 << id) });
}

fn channel_busy<C: SingleChannel>(channel: &C) -> bool {
    channel.ch().ch_ctrl_trig().read().busy().bit_is_set()
}

fn channel_has_bus_error<C: SingleChannel>(channel: &C) -> bool {
    channel.ch().ch_ctrl_trig().read().ahb_error().bit_is_set()
}

fn publish_chain<C: SingleChannel>(channel: &C, next_channel: u8) -> Result<(), RefillError> {
    let registers = channel.ch();
    // El PAC enmascara READ_ERROR y WRITE_ERROR (W1C) durante modify.
    registers.ch_al1_ctrl().modify(|_, w| w.en().clear_bit());
    cortex_m::asm::dsb();

    if channel_has_bus_error(channel) {
        return Err(RefillError::DmaBusError);
    }
    if !channel_busy(channel) || channel_completed(channel.id()) {
        return Err(RefillError::DeadlineMissed);
    }

    // SAFETY: el índice corresponde al otro canal del par, ya configurado.
    registers
        .ch_al1_ctrl()
        .modify(|_, w| unsafe { w.chain_to().bits(next_channel).en().set_bit() });
    Ok(())
}

fn refill_channel<C, P, I>(
    channel: &C,
    previous: &P,
    buffer: &mut DmaCommandBuffer,
    tx: &StepDirTx,
    reader: &mut PioCommandBlockReader<I>,
) -> Result<Option<PioCommandBlock>, RefillError>
where
    C: SingleChannel,
    P: SingleChannel,
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    if !channel_completed(channel.id()) || channel_busy(channel) {
        return Ok(None);
    }
    if channel_completed(previous.id()) {
        return Err(RefillError::DeadlineMissed);
    }
    if !channel_busy(previous) {
        return Ok(None);
    }

    cortex_m::asm::dsb();
    compiler_fence(Ordering::SeqCst);
    let block = buffer
        .fill_from(reader)
        .map_err(RefillError::Source)?
        .ok_or(RefillError::MissingBlock)?;

    clear_channel_completion(channel.id());
    configure_channel(channel, buffer, tx, channel.id());

    compiler_fence(Ordering::SeqCst);
    cortex_m::asm::dsb();
    publish_chain(previous, channel.id())?;
    Ok(Some(block))
}

fn configure_channel<C: SingleChannel>(
    channel: &C,
    buffer: &DmaCommandBuffer,
    tx: &StepDirTx,
    chain_to: u8,
) {
    let registers = channel.ch();
    // SAFETY: los valores son DMA de 32 bits, el DREQ de la FIFO propia
    // y un índice de canal del par. Esta configuración no dispara DMA.
    registers.ch_al1_ctrl().write(|w| unsafe {
        w.data_size()
            .bits(2)
            .incr_read()
            .set_bit()
            .incr_write()
            .clear_bit()
            .treq_sel()
            .bits(tx.dreq_value())
            .chain_to()
            .bits(chain_to)
            .bswap()
            .clear_bit()
            .en()
            .set_bit()
    });
    // SAFETY: el búfer es estático, alineado a u32 y acotado a 28 bits.
    // La dirección de escritura es la FIFO TX poseída por la transferencia.
    registers
        .ch_read_addr()
        .write(|w| unsafe { w.bits(buffer.as_slice().as_ptr() as u32) });
    registers
        .ch_write_addr()
        .write(|w| unsafe { w.bits(tx.fifo_address() as u32) });
    registers
        .ch_trans_count()
        .write(|w| unsafe { w.count().bits(buffer.len() as u32).mode().normal() });
}

fn clear_channel_errors<C: SingleChannel>(channel: &C) {
    // Reconocer los errores después de detener el canal y registrar su estado.
    channel.ch().ch_al1_ctrl().modify(|_, w| {
        w.read_error()
            .clear_bit_by_one()
            .write_error()
            .clear_bit_by_one()
    });
}

fn disable_channel<C: SingleChannel>(channel: &C) {
    // SAFETY: apuntar al propio canal deshabilita el encadenamiento. El alias
    // no dispara DMA y modify conserva la configuración y los flags W1C.
    channel
        .ch()
        .ch_al1_ctrl()
        .modify(|_, w| unsafe { w.chain_to().bits(channel.id()).en().clear_bit() });
}
