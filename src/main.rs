use std::{collections::VecDeque, env, io::Read, sync::mpsc, thread, time::Duration};

use ndarray::{Array2, array};
use rand::prelude::*;

type Position = (usize, usize);

fn pathplan(grid: &Array2<i32>, t: i32, start_pos: Position) -> Result<(usize, i32, Position), ()> {
    let mut grid = grid.clone();

    let n = grid.shape()[0];

    let mut rng = rand::rng();

    let mut current_cost = 0;
    let mut current_step = 0;
    let mut current_pos = start_pos;

    let mut trail = VecDeque::new();
    trail.push_back(current_pos);

    for _ in 0..t {
        grid[current_pos] += 1;

        println!("Drone pos is now: {:?}", current_pos);

        let (x, y) = current_pos;
        let neighbors = [
            (x.saturating_sub(1), y.saturating_sub(1)),
            (x.saturating_sub(1), y),
            (x.saturating_sub(1), y.saturating_add(1).min(n - 1)),
            (x, y.saturating_sub(1)),
            (x, y.saturating_add(1).min(n - 1)),
            (x.saturating_add(1).min(n - 1), y.saturating_sub(1)),
            (x.saturating_add(1).min(n - 1), y),
            (
                x.saturating_add(1).min(n - 1),
                y.saturating_add(1).min(n - 1),
            ),
        ];

        assert!(!neighbors.is_empty());

        let valid_neighbors: Vec<_> = neighbors
            .iter()
            .filter(|&&pos| pos != current_pos && !trail.contains(&pos))
            .collect();
        let lowest_neighbor = match valid_neighbors.iter().min_by_key(|&&&pos| grid[pos]) {
            Some(neighbor) => **neighbor,
            None => neighbors
                .iter()
                .choose(&mut rng)
                .map_or(current_pos, |&n| n),
        };

        trail.push_back(current_pos);
        trail.pop_front();
        current_cost += grid[lowest_neighbor];
        current_step += 1;
        current_pos = lowest_neighbor;

        // NOTE: Simulating additional CPU time
        std::thread::sleep(std::time::Duration::from_millis(1_000));
    }

    Ok((current_step, current_cost, current_pos))
}

fn run_pathplan_with_timeout(
    grid: &Array2<i32>,
    t: i32,
    start_pos: Position,
    timeout_ms: u64,
) -> Option<Result<(usize, i32, Position), ()>> {
    let grid_clone = grid.clone();

    let (tx, rx) = mpsc::channel();

    let handle = thread::spawn(move || {
        let result = pathplan(&grid_clone, t, start_pos);
        let _ = tx.send(result);
        result
    });

    match rx.recv_timeout(Duration::from_millis(timeout_ms)) {
        Ok(result) => Some(result),
        Err(_) => {
            handle.join().ok();
            None
        }
    }
}

fn grid_from_file<P: AsRef<std::path::Path>>(p: P) -> std::io::Result<Array2<i32>> {
    let mut grid_file = std::fs::File::open(p)?;

    // NOTE: IRL we would never commit an entire file to memory like this
    let mut s = String::new();
    grid_file.read_to_string(&mut s)?;

    let lines: Vec<&str> = s.trim().lines().collect();

    let rows = lines.len();
    let cols = lines
        .get(0)
        .map_or(0, |line| line.split_whitespace().count());

    if rows == 0 || cols == 0 {
        Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "empty grid",
        ))?;
    }

    let mut grid = Array2::zeros((rows, cols));

    for (i, line) in lines.iter().enumerate() {
        for (j, val) in line.split_whitespace().enumerate() {
            if j < cols {
                grid[[i, j]] = val.parse::<i32>().unwrap_or(0);
            }
        }
    }

    Ok(grid)
}

fn main() {
    let args: Vec<String> = env::args().collect();

    let grid = if args.len() > 1 {
        grid_from_file(&args[1]).expect("failed to read grid from file")
    } else {
        array![
            [01, 05, 05, 02, 05],
            [01, 02, 03, 04, 02],
            [03, 01, 00, 04, 00],
            [09, 05, 01, 06, 01],
            [00, 02, 06, 01, 03]
        ]
    };

    let start_pos = (1, 2);
    let time_steps = 7;
    let timeout_ms = 5_000; // 5s

    match run_pathplan_with_timeout(&grid, time_steps, start_pos, timeout_ms) {
        Some(Ok((fin_step, fin_cost, fin_pos))) => {
            println!("Cost after {} iterations: {}", fin_step, fin_cost);
            println!("Position after {} iterations: {:?}", fin_step, fin_pos);
        }
        _ => {
            println!("Pathplan did not complete within the time limit");
        }
    }
}
