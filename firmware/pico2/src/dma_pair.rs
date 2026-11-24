use core::sync::atomic::{Ordering, compiler_fence};

use rp235x_hal::{
    dma::{CH0, CH1, Channel, SingleChannel},
    pac::{self, PIO0},
    pio::{SM0, Tx},
};

use crate::dma_command_buffer::DmaCommandBuffer;

type StepDirTx = Tx<(PIO0, SM0)>;
const CHANNEL_MASK: u16 = 0b11;

/// Transfiere dos bloques a una misma FIFO, con CH0 encadenado a CH1.
#[must_use = "llama a stop para recuperar los canales, los búferes y la FIFO"]
pub struct DmaPairTransfer {
    channel_a: Channel<CH0>,
    channel_b: Channel<CH1>,
    buffer_a: DmaCommandBuffer,
    buffer_b: DmaCommandBuffer,
    tx: StepDirTx,
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
        assert!(!buffer_a.is_empty());
        assert!(!buffer_b.is_empty());
        assert!(!channel_a.ch().ch_ctrl_trig().read().busy().bit_is_set());
        assert!(!channel_b.ch().ch_ctrl_trig().read().busy().bit_is_set());

        // Elegir el propio canal en CHAIN_TO deshabilita el encadenamiento.
        let last_channel = channel_b.id();
        configure_channel(&channel_b, &buffer_b, &tx, last_channel);
        configure_channel(&channel_a, &buffer_a, &tx, last_channel);

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
        }
    }

    pub fn channel_a_busy(&self) -> bool {
        self.channel_a
            .ch()
            .ch_ctrl_trig()
            .read()
            .busy()
            .bit_is_set()
    }

    pub fn channel_b_busy(&self) -> bool {
        self.channel_b
            .ch()
            .ch_ctrl_trig()
            .read()
            .busy()
            .bit_is_set()
    }

    pub fn remaining_words(&self) -> [u32; 2] {
        [
            remaining_words(&self.channel_a, &self.buffer_a),
            remaining_words(&self.channel_b, &self.buffer_b),
        ]
    }

    pub fn has_bus_error(&self) -> bool {
        self.channel_a
            .ch()
            .ch_ctrl_trig()
            .read()
            .ahb_error()
            .bit_is_set()
            || self
                .channel_b
                .ch()
                .ch_ctrl_trig()
                .read()
                .ahb_error()
                .bit_is_set()
    }

    /// Indica que los dos bloques se transfirieron a la FIFO.
    pub fn is_done(&self) -> bool {
        self.remaining_words() == [0, 0]
            && !self.channel_a_busy()
            && !self.channel_b_busy()
            && !self.has_bus_error()
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

fn configure_channel<C: SingleChannel>(
    channel: &C,
    buffer: &DmaCommandBuffer,
    tx: &StepDirTx,
    chain_to: u8,
) {
    let registers = channel.ch();
    // SAFETY: los valores son DMA de 32 bits, el DREQ de la FIFO propia
    // y el índice de CH1. Los registros de esta configuración no disparan DMA.
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
            .read_error()
            .clear_bit_by_one()
            .write_error()
            .clear_bit_by_one()
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

fn remaining_words<C: SingleChannel>(channel: &C, buffer: &DmaCommandBuffer) -> u32 {
    let registers = channel.ch();
    // TRANS_COUNT escribe la recarga, pero al leer devuelve el conteo activo.
    // Si la dirección no ha avanzado, el bloque entero sigue pendiente.
    if registers.ch_read_addr().read().bits() == buffer.as_slice().as_ptr() as u32 {
        buffer.len() as u32
    } else {
        registers.ch_trans_count().read().count().bits()
    }
}

fn disable_channel<C: SingleChannel>(channel: &C) {
    // SAFETY: apuntar al propio canal deshabilita el encadenamiento. El alias
    // no dispara DMA y modify conserva la configuración y los flags W1C.
    channel
        .ch()
        .ch_al1_ctrl()
        .modify(|_, w| unsafe { w.chain_to().bits(channel.id()).en().clear_bit() });
}
