use std::collections::VecDeque;

use ndarray::{Array2, array};
use rand::prelude::*;

// - N: Grootte van het te doorzoeken grid: N x N.
// - t: Totaal aantal discrete tijdstappen
// - T: Maximale tijdsduur van het algoritme in milliseconden.
// - (x, y): Index van startpositie van de drone

type Position = (usize, usize);

fn pathplan(grid: &Array2<i32>, t: i32, start_pos: Position) -> (usize, i32, Position) {
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

        println!("Current pos is now: {:?}", current_pos);

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

        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    (current_step, current_cost, current_pos)
}

fn main() {
    let grid = array![
        [01, 05, 05, 02, 05],
        [01, 02, 03, 04, 02],
        [03, 01, 00, 04, 00],
        [09, 05, 01, 06, 01],
        [00, 02, 06, 01, 03]
    ];

    let start_pos = (1, 2);
    let (fin_step, fin_cost, fin_pos) = pathplan(&grid, 7, start_pos);

    println!("Cost after {} iterations: {}", fin_step, fin_cost);
    println!("Position after {} iterations: {:?}", fin_step, fin_pos);
}
