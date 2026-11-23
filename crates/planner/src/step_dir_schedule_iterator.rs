use core::iter::FusedIterator;

use crate::{PlannerError, ScheduledStepDirEvent, StepDirEvent, StepDirScheduleCursor};

/// Programa eventos bajo demanda y termina tras el primer error o el agotamiento de la fuente.
pub struct StepDirScheduleIterator<I> {
    source: I,
    cursor: StepDirScheduleCursor,
    finished: bool,
}

impl<I> StepDirScheduleIterator<I>
where
    I: Iterator<Item = Result<StepDirEvent, PlannerError>>,
{
    pub fn new(source: I, cursor: StepDirScheduleCursor) -> Self {
        Self {
            source,
            cursor,
            finished: false,
        }
    }
}

impl<I> Iterator for StepDirScheduleIterator<I>
where
    I: Iterator<Item = Result<StepDirEvent, PlannerError>>,
{
    type Item = Result<ScheduledStepDirEvent, PlannerError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }

        let Some(event) = self.source.next() else {
            self.finished = true;
            return None;
        };

        let scheduled = event.and_then(|event| self.cursor.schedule(event));
        if scheduled.is_err() {
            self.finished = true;
        }
        Some(scheduled)
    }
}

impl<I> FusedIterator for StepDirScheduleIterator<I> where
    I: Iterator<Item = Result<StepDirEvent, PlannerError>>
{
}
