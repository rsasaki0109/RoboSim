//! Small dense linear-algebra helpers for the optimal-control solvers.

/// Matrix-vector product for a row-major matrix.
pub fn mat_vec(matrix: &[Vec<f64>], vector: &[f64]) -> Vec<f64> {
    matrix
        .iter()
        .map(|row| row.iter().zip(vector).map(|(a, b)| a * b).sum())
        .collect()
}

/// Matrix product `a * b`.
pub fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let columns = b.first().map_or(0, Vec::len);
    let mut out = vec![vec![0.0; columns]; a.len()];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, cell) in out_row.iter_mut().enumerate() {
            *cell = (0..b.len()).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

/// Transpose of a row-major matrix.
pub fn mat_transpose(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rows = a.len();
    let columns = a.first().map_or(0, Vec::len);
    let mut out = vec![vec![0.0; rows]; columns];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, cell) in out_row.iter_mut().enumerate() {
            *cell = a[column][row];
        }
    }
    out
}

/// `a + b`.
pub fn mat_add(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    a.iter()
        .zip(b)
        .map(|(row_a, row_b)| row_a.iter().zip(row_b).map(|(x, y)| x + y).collect())
        .collect()
}

/// `a - b`.
pub fn mat_sub(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    a.iter()
        .zip(b)
        .map(|(row_a, row_b)| row_a.iter().zip(row_b).map(|(x, y)| x - y).collect())
        .collect()
}

/// `matrix * factor`.
pub fn mat_scale(matrix: &[Vec<f64>], factor: f64) -> Vec<Vec<f64>> {
    matrix
        .iter()
        .map(|row| row.iter().map(|value| value * factor).collect())
        .collect()
}

/// Identity matrix.
pub fn identity(size: usize) -> Vec<Vec<f64>> {
    let mut out = vec![vec![0.0; size]; size];
    for (index, row) in out.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    out
}

/// Symmetrizes a square matrix as `0.5 * (m + m^T)`.
pub fn symmetrize(matrix: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let transpose = mat_transpose(matrix);
    mat_scale(&mat_add(matrix, &transpose), 0.5)
}

/// Dot product.
pub fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Adds `value` to the diagonal.
pub fn add_diagonal(matrix: &mut [Vec<f64>], value: f64) {
    for (index, row) in matrix.iter_mut().enumerate() {
        row[index] += value;
    }
}

/// Inverts a square matrix by Gauss-Jordan elimination, returning `None` when
/// the matrix is singular.
pub fn invert(matrix: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = matrix.len();
    if n == 0 || matrix.iter().any(|row| row.len() != n) {
        return None;
    }

    let mut work = matrix.to_vec();
    let mut inverse = identity(n);
    for column in 0..n {
        let pivot =
            (column..n).max_by(|&a, &b| work[a][column].abs().total_cmp(&work[b][column].abs()))?;
        if work[pivot][column].abs() < 1.0e-12 {
            return None;
        }
        work.swap(column, pivot);
        inverse.swap(column, pivot);
        let diagonal = work[column][column];
        for value in work[column].iter_mut() {
            *value /= diagonal;
        }
        for value in inverse[column].iter_mut() {
            *value /= diagonal;
        }
        let pivot_row = work[column].clone();
        let pivot_inverse = inverse[column].clone();
        for row in 0..n {
            if row == column {
                continue;
            }
            let factor = work[row][column];
            if factor == 0.0 {
                continue;
            }
            for (target, source) in work[row].iter_mut().zip(&pivot_row) {
                *target -= factor * source;
            }
            for (target, source) in inverse[row].iter_mut().zip(&pivot_inverse) {
                *target -= factor * source;
            }
        }
    }
    Some(inverse)
}
