use crate::{ExecutorError, PioCommandWord};

/// Metadatos de las palabras escritas en el búfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PioCommandBlock {
    pub len: usize,
    pub is_final: bool,
}

/// Divide una secuencia PIO en bloques y conserva las palabras originales.
pub struct PioCommandBlockReader<I> {
    source: I,
    finished: bool,
}

impl<I> PioCommandBlockReader<I>
where
    I: Iterator<Item = Result<PioCommandWord, ExecutorError>>,
{
    pub fn new(source: I) -> Self {
        Self {
            source,
            finished: false,
        }
    }

    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// El prefijo de longitud `len` está listo para usarse si devuelve un bloque.
    /// Un búfer vacío devuelve un error recuperable sin avanzar la fuente
    /// mientras el lector está activo. Un lector agotado devuelve `Ok(None)`.
    /// Los errores de la fuente o de la secuencia agotan el lector y pueden
    /// dejar palabras parciales en `output`, que deben descartarse.
    pub fn next_block(
        &mut self,
        output: &mut [u32],
    ) -> Result<Option<PioCommandBlock>, ExecutorError> {
        if self.finished {
            return Ok(None);
        }

        if output.is_empty() {
            return Err(ExecutorError::EmptyCommandBuffer);
        }

        for (index, destination) in output.iter_mut().enumerate() {
            let command = match self.source.next() {
                Some(Ok(command)) => command,
                Some(Err(error)) => {
                    self.finished = true;
                    return Err(error);
                }
                None => {
                    self.finished = true;
                    return Err(ExecutorError::MissingEndCommand);
                }
            };
            *destination = command.bits();

            if command.is_end() {
                self.finished = true;
                match self.source.next() {
                    None => {}
                    Some(Ok(_)) => return Err(ExecutorError::UnexpectedCommandAfterEnd),
                    Some(Err(error)) => return Err(error),
                }

                return Ok(Some(PioCommandBlock {
                    len: index + 1,
                    is_final: true,
                }));
            }
        }

        Ok(Some(PioCommandBlock {
            len: output.len(),
            is_final: false,
        }))
    }
}
