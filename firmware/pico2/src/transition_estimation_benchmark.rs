use core::hint::black_box;

use cortex_m::peripheral::DWT;
use planner::{
    CachedTrajectoryPositionSampler, CoordinatedJointTrajectory, JointToStepsConverter,
    PlannerError, StepTimeBase, TimedCoordinatedJointPathProfiles, TransitionEstimationStatistics,
    find_first_transition_tick_exponential, find_first_transition_tick_hinted,
    generate_guarded_segment_boundaries,
};

pub fn enable_cycle_counter() {
    let mut peripherals = cortex_m::Peripherals::take().unwrap();
    peripherals.DCB.enable_trace();
    assert!(DWT::has_cycle_counter());
    DWT::unlock();
    peripherals.DWT.enable_cycle_counter();

    let start = DWT::cycle_count();
    cortex_m::asm::delay(64);
    assert_ne!(DWT::cycle_count(), start);
}

pub fn measure(
    profiles: TimedCoordinatedJointPathProfiles<'_>,
    converter: JointToStepsConverter,
    timebase: StepTimeBase,
) -> Result<(), PlannerError> {
    let mut boundaries = [0; 2];
    generate_guarded_segment_boundaries(profiles, timebase, &mut boundaries)?;
    let [mut tick, end_tick] = boundaries;
    let mut segments = profiles;
    let profile = segments.next().ok_or(PlannerError::InsufficientPoints)??;
    assert!(segments.next().is_none(), "el benchmark usa un segmento");

    let trajectory = CoordinatedJointTrajectory::new(profiles)?;
    let mut exponential_sampler = CachedTrajectoryPositionSampler::new(trajectory);
    let mut hinted_sampler = CachedTrajectoryPositionSampler::new(trajectory);
    let mut position = converter
        .convert(exponential_sampler.position_at_clamped(timebase.ticks_to_seconds(tick))?)?;
    hinted_sampler.position_at_clamped(timebase.ticks_to_seconds(tick))?;

    let mut stats = TransitionEstimationStatistics::default();
    let mut estimate_cycles = 0u64;
    let mut max_estimate_cycles = 0u32;
    let mut exponential_cycles = 0u64;
    let mut hinted_cycles = 0u64;
    let mut exponential_samples = 0u64;
    let mut hinted_samples = 0u64;
    let mut max_hinted_samples = 0u64;

    while tick < end_tick {
        let (estimate, estimate_elapsed) =
            measure_cycles(|| profile.estimate_next_step_tick(&converter, position, timebase));
        let mut baseline_probes = 0;
        let (expected, exponential_elapsed) = measure_cycles(|| {
            find_first_transition_tick_exponential(tick, end_tick, position, |probe| {
                baseline_probes += 1;
                let angles =
                    exponential_sampler.position_at_clamped(timebase.ticks_to_seconds(probe))?;
                converter.convert(angles)
            })
        });
        let mut hinted_probes = 0;
        let (actual, hinted_elapsed) = measure_cycles(|| {
            find_first_transition_tick_hinted(tick, end_tick, position, estimate, |probe| {
                hinted_probes += 1;
                let angles =
                    hinted_sampler.position_at_clamped(timebase.ticks_to_seconds(probe))?;
                converter.convert(angles)
            })
        });
        let expected = expected?;
        assert_eq!(
            expected, actual?,
            "busquedas distintas desde el tick {tick}"
        );
        let Some((actual_tick, _)) = expected else {
            break;
        };

        stats.record(actual_tick, estimate);
        estimate_cycles += u64::from(estimate_elapsed);
        max_estimate_cycles = max_estimate_cycles.max(estimate_elapsed);
        exponential_cycles += u64::from(exponential_elapsed);
        hinted_cycles += u64::from(hinted_elapsed);
        exponential_samples += baseline_probes;
        hinted_samples += hinted_probes;
        max_hinted_samples = max_hinted_samples.max(hinted_probes);

        tick = actual_tick;
        let angles = exponential_sampler.position_at_clamped(timebase.ticks_to_seconds(tick))?;
        position = converter.convert(angles)?;
    }

    defmt::info!(
        "Estimacion: transiciones={}, estimaciones={}, exactas={}, hasta_un_tick={}, hasta_dos_ticks={}, error_maximo_ticks={}",
        stats.transitions,
        stats.estimates,
        stats.exact_estimates,
        stats.estimates_within_one_tick,
        stats.estimates_within_two_ticks,
        stats.max_error_ticks,
    );
    defmt::info!(
        "Coste: estimacion_ciclos={}, estimacion_maxima_ciclos={}, exponencial_ciclos={}, busqueda_con_estimacion_ciclos={}, muestras_exponencial={}, muestras_con_estimacion={}, maximo_muestras_con_estimacion={}",
        estimate_cycles,
        max_estimate_cycles,
        exponential_cycles,
        hinted_cycles,
        exponential_samples,
        hinted_samples,
        max_hinted_samples,
    );
    Ok(())
}

fn measure_cycles<T>(operation: impl FnOnce() -> T) -> (T, u32) {
    let start = DWT::cycle_count();
    let result = black_box(black_box(operation)());
    (result, DWT::cycle_count().wrapping_sub(start))
}
