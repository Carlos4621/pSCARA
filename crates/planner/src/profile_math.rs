// Small products of finite factors are evaluated as mantissas and powers of
// two. Only the final result is rounded into the f64 exponent range.
// All denominators at the call sites are strictly positive.
fn ratio_parts(numerators: &[f64], denominators: &[f64]) -> (f64, i32) {
    let mut fraction = 1.0;
    let mut exponent = 0;

    for (index, &value) in numerators.iter().enumerate() {
        // Cancel equal factors, preserving exact identity rescalings.
        let occurrence = numerators[..index].iter().filter(|&&x| x == value).count();
        if occurrence < denominators.iter().filter(|&&x| x == value).count() {
            continue;
        }
        let (part, power) = libm::frexp(value);
        fraction *= part;
        exponent += power;
    }
    for (index, &value) in denominators.iter().enumerate() {
        let occurrence = denominators[..index]
            .iter()
            .filter(|&&x| x == value)
            .count();
        if occurrence < numerators.iter().filter(|&&x| x == value).count() {
            continue;
        }
        let (part, power) = libm::frexp(value);
        fraction /= part;
        exponent -= power;
    }
    (fraction, exponent)
}

pub(crate) fn product_ratio(numerators: &[f64], denominators: &[f64]) -> f64 {
    let (fraction, exponent) = ratio_parts(numerators, denominators);
    libm::scalbn(fraction, exponent)
}

pub(crate) fn sqrt_product_ratio(numerators: &[f64], denominators: &[f64]) -> f64 {
    let (mut fraction, mut exponent) = ratio_parts(numerators, denominators);
    if exponent % 2 != 0 {
        fraction *= 2.0;
        exponent -= 1;
    }
    libm::scalbn(libm::sqrt(fraction), exponent / 2)
}

// Compare products of finite f64 factors without rounding either product.
// A significand has at most 53 bits, so its exact product fits in a u128.
pub(crate) fn products_equal(left: [f64; 2], right: [f64; 2]) -> bool {
    fn parts(factors: [f64; 2]) -> (bool, u128, i32) {
        debug_assert!(factors.iter().all(|factor| factor.is_finite()));
        if factors.contains(&0.0) {
            return (false, 0, 0);
        }

        let mut significand = 1_u128;
        let mut exponent = 0;
        for factor in factors {
            let bits = factor.to_bits();
            let stored_exponent = ((bits >> 52) & 0x7ff) as i32;
            let fraction = bits & ((1_u64 << 52) - 1);
            let (mantissa, power) = if stored_exponent == 0 {
                (fraction, -1074)
            } else {
                (fraction | (1_u64 << 52), stored_exponent - 1075)
            };
            significand *= u128::from(mantissa);
            exponent += power;
        }

        // Canonicalize equivalent powers of two, including subnormal factors.
        let shift = significand.trailing_zeros();
        (
            factors[0].is_sign_negative() ^ factors[1].is_sign_negative(),
            significand >> shift,
            exponent + shift as i32,
        )
    }

    parts(left) == parts(right)
}

#[cfg(test)]
#[path = "../tests/unit/profile_math.rs"]
mod tests;
