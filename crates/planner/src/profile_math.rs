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

#[cfg(test)]
#[path = "../tests/unit/profile_math.rs"]
mod tests;
