use std::{
    cmp::Reverse,
    collections::{BTreeSet, BinaryHeap},
    error::Error,
    fmt,
};

pub type Map<T> = Vec<Vec<T>>;

#[derive(PartialEq, Clone)]
pub struct Point {
    pub x: usize,
    pub y: usize,
}

impl Point {
    pub fn new(x: usize, y: usize) -> Self {
        Self { x, y }
    }

    fn from_direction(point: &FPoint, direction: (i8, i8)) -> Self {
        let (x, y) = direction;

        let new_x = match x {
            1 => point.x.saturating_add(1),
            -1 => point.x.saturating_sub(1),
            _ => point.x,
        };

        let new_y = match y {
            1 => point.y.saturating_add(1),
            -1 => point.y.saturating_sub(1),
            _ => point.y,
        };

        Self { x: new_x, y: new_y }
    }
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

#[derive(Clone)]
struct FPoint {
    x: usize,
    y: usize,
    g: f64,
    f: f64,
    parent: Option<Point>,
}

impl FPoint {
    pub fn with_f(point: &Point, g: f64, h: f64, parent: Point) -> Self {
        Self {
            x: point.x,
            y: point.y,
            g,
            f: h + g,
            parent: Some(parent),
        }
    }
}

impl From<&Point> for FPoint {
    fn from(value: &Point) -> Self {
        Self {
            x: value.x,
            y: value.y,
            g: 0.0,
            f: 0.0,
            parent: None,
        }
    }
}

impl TryInto<Point> for FPoint {
    type Error = ();

    fn try_into(self) -> Result<Point, Self::Error> {
        Ok(Point {
            x: self.x,
            y: self.y,
        })
    }
}

impl PartialEq<Point> for FPoint {
    fn eq(&self, other: &Point) -> bool {
        (self.x == other.x) & (self.y == other.y)
    }
}

impl Eq for FPoint {}

impl PartialEq for FPoint {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl PartialOrd for FPoint {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

impl Ord for FPoint {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.f.total_cmp(&other.f)
    }
}

pub struct Validator<T: Ord> {
    valid_values: BTreeSet<T>,
}

impl<T> Validator<T>
where
    T: Ord,
{
    fn check(&self, map: &Map<T>, point: &Point) -> bool {
        ((point.x < map[0].len()) & (point.y < map.len()))
            && self.valid_values.contains(&map[point.y][point.x])
    }
}

impl<T> From<T> for Validator<T>
where
    T: Ord,
{
    fn from(value: T) -> Self {
        Self {
            valid_values: BTreeSet::from([value]),
        }
    }
}

impl<T, const N: usize> From<[T; N]> for Validator<T>
where
    T: Ord,
{
    fn from(value: [T; N]) -> Self {
        Self {
            valid_values: BTreeSet::from(value),
        }
    }
}

fn calculate_h(current: &Point, goal: &Point) -> f64 {
    let dx = current.x.abs_diff(goal.x) as f64;
    let dy = current.y.abs_diff(goal.y) as f64;
    dx.max(dy)
}

type ClosedListElement = (f64, Option<Point>);

#[derive(Debug)]
pub enum AStarError {
    InvalidStartPoint,
    InvalidGoalPoint,
    NotFoundPath,
}

impl fmt::Display for AStarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match *self {
                AStarError::InvalidStartPoint =>
                    "The start point is invalid. Please provide a valid starting position.",
                AStarError::InvalidGoalPoint =>
                    "The goal point is invalid. Please provide a valid destination.",
                AStarError::NotFoundPath =>
                    "No valid path was found between the start and goal points.",
            }
        )
    }
}

impl Error for AStarError {}

pub fn a_star<T: Ord>(
    map: &Map<T>,
    start: &Point,
    goal: &Point,
    validator: &Validator<T>,
) -> Result<Vec<Point>, AStarError> {
    if !validator.check(&map, &start) {
        return Err(AStarError::InvalidStartPoint);
    }
    if !validator.check(&map, &goal) {
        return Err(AStarError::InvalidGoalPoint);
    }
    if start == goal {
        return Ok(vec![]);
    }

    let mut open_list = BinaryHeap::new();
    let mut closed_list = vec![vec![(0.0, None); map[0].len()]; map.len()];

    let directions = [
        (0, 1),
        (-1, 0),
        (0, -1),
        (1, 0),
        (1, 1),
        (1, -1),
        (-1, -1),
        (-1, 1),
    ];

    open_list.push(Reverse(FPoint::from(start)));

    while !open_list.is_empty() {
        let point = open_list.pop().unwrap().0;

        for direction in directions {
            let succescor = Point::from_direction(&point, direction);

            if validator.check(&map, &succescor) {
                if &succescor == goal {
                    closed_list[point.y][point.x].1 = point.parent.clone();
                    closed_list[goal.y][goal.x].1 = Some(point.try_into().unwrap());
                    return Ok(reverse_path(closed_list, start, goal));
                } else {
                    let succescor = FPoint::with_f(
                        &succescor,
                        point.g + 1.0,
                        calculate_h(&succescor, &goal),
                        point.to_owned().try_into().unwrap(),
                    );

                    if (closed_list[succescor.y][succescor.x].0 != 0.0)
                        & (closed_list[succescor.y][succescor.x].0 < succescor.f)
                    {
                        continue;
                    }

                    closed_list[succescor.y][succescor.x].0 = succescor.f;
                    closed_list[succescor.y][succescor.x].1 = succescor.parent.clone();
                    open_list.push(Reverse(succescor));
                }
            } else {
                continue;
            }
        }
        closed_list[point.y][point.x].0 = point.f;
    }
    Err(AStarError::NotFoundPath)
}

fn reverse_path(closed_list: Vec<Vec<ClosedListElement>>, start: &Point, goal: &Point) -> Vec<Point> {
    let mut path: Vec<Point> = Vec::new();
    path.push(goal.to_owned());
    let mut point = closed_list[goal.y][goal.x].1.clone();

    loop {
        if let Some(p) = point {
            if start == &p {
                break;
            }
            path.push(p.to_owned());
            point = closed_list[p.y][p.x].1.clone();
        } else {
            break;
        }
    }
    path.reverse();
    path
}
