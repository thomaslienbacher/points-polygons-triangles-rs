extern crate core;

use std::ops::Div;
use std::thread;
use std::time::Duration;

use cgmath::{Matrix3, vec2, Vector2};
use cgmath::num_traits::FloatConst;
use cgmath::prelude::*;
use rand_distr::Normal;

use rand::{Rng, thread_rng};
use rand::distributions::{Standard, Uniform};
use svg::{Document, node};
use svg::node::element::{Circle, Group, Path, Rectangle, Text};
use svg::node::element::path::Data;

use crate::Orientation::*;

type Point = Vector2<f64>;
type Mat = Matrix3<f64>;

#[derive(Debug, Clone)]
struct ConvexPoly {
    all: Vec<Point>,
    hull: Vec<Point>,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
}

impl ConvexPoly {
    pub fn new(mut points: Vec<Point>) -> Self {
        let mut hull = vec![];
        points.sort_by(|a, b| a.y.total_cmp(&b.y));
        hull.push(points[0]);

        points[1..].sort_by(|a, b| {
            let smallest = hull[0].clone();
            let aa = (*a - smallest);
            let ba = (*b - smallest);
            angle(&aa).total_cmp(&angle(&ba)).reverse()
        });

        let mut all = points.clone();
        let n = points.len();

        for i in 1..n {
            let p = &points[i];
            while hull.len() > 1 &&
                Orientation::calc(&hull[hull.len() - 2], &hull[hull.len() - 1], p) == Leftwards {
                hull.pop();
            }

            hull.push(p.clone());
        }

        // TODO: find min max during hull construction or sorting
        ConvexPoly {
            all,
            hull,
            x_min: points.iter().map(|p| p.x).reduce(|a, b| a.min(b)).unwrap(),
            x_max: points.iter().map(|p| p.x).reduce(|a, b| a.max(b)).unwrap(),
            y_min: points.iter().map(|p| p.y).reduce(|a, b| a.min(b)).unwrap(),
            y_max: points.iter().map(|p| p.y).reduce(|a, b| a.max(b)).unwrap(),
        }
    }
}

fn angle(p: &Point) -> f64 {
    f64::atan2(p.y, p.x)
}

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
enum Orientation {
    Leftwards,
    Collinear,
    Rightwards,
}

impl Orientation {
    fn calc(s: &Point, p: &Point, e: &Point) -> Orientation {
        let m = Mat::new(
            1.0, 1.0, 1.0,
            s.x, p.x, e.x,
            s.y, p.y, e.y,
        );
        let sign = m.determinant();

        if sign < 0.0 {
            return Rightwards;
        }

        if sign > 0.0 {
            return Leftwards;
        }

        Collinear
    }
}

fn is_point_in_polygon(poly: &ConvexPoly, p: &Point) -> bool {
    if p.x < poly.x_min || p.x > poly.x_max || p.y < poly.y_min || p.y > poly.y_max {
        return false;
    }

    let hull = &poly.hull;

    for i in 0..hull.len() {
        let s = hull.get(i).unwrap();
        let e = hull.get((i + 1) % hull.len()).unwrap();
        if Orientation::calc(s, p, e) == Rightwards {
            return false;
        }
    }

    true
}


fn binary_search_angles(points: &[Point], low: usize, high: usize, center: &Point, search_angle: f64) -> usize {
    //println!("checking from {low} to {high} with search angle {search_angle}");
    assert!(low <= high);
    if low == high {
        return low;
    }

    if high - low == 1 {
        let lowdiff = (angle(&(points[low] - center)) - search_angle).abs();
        let highdiff = (angle(&(points[high] - center)) - search_angle).abs();
        //println!("low diff: {lowdiff}");
        //println!("high diff: {highdiff}");

        if lowdiff < highdiff {
            return low;
        } else {
            return high;
        }
    }

    let middle = ((high + low) / 2);
    let mut middle_angle = angle(&(points[middle] - center));

    //println!("angle of {middle} is {:02.3}", middle_angle);

    if middle_angle < search_angle {
        if middle - 1 == high {
            panic!("infinite recursion");
        }
        return binary_search_angles(points, low, middle, center, search_angle);
    } else {
        if middle + 1 == low {
            panic!("infinite recursion");
        }
        return binary_search_angles(points, middle, high, center, search_angle);
    }
}

fn is_point_in_polygon_fast(poly: &ConvexPoly, p: &Point) -> bool {
    //if p.x < poly.x_min || p.x > poly.x_max || p.y < poly.y_min || p.y > poly.y_max {
    //    return false;
    //}

    let center = poly.hull[0];
    let mut search_angle = angle(&(p - center));
    if search_angle < 0.0 {
        //println!("search angle negative");
        search_angle += 2.0 * f64::PI();
    }

    /*println!("searching for: {:02.3}", search_angle);

    print!("angles: ");
    for a in &poly.hull[1..] {
        let mut ang = angle(&(a - center));
        print!("{:02.3} ", ang)
    }
    println!();*/

    // check angles around center
    let center_left = angle(&(poly.hull.last().unwrap() - center));
    let center_right = angle(&(poly.hull[1] - center));
    //println!("center left: {center_left}");
    //println!("center right: {center_right}");

    if search_angle > center_right || search_angle < center_left {
        return false;
    }

    // binary search the two nodes whose angles are the nearest to `angle`
    // this only works because hull is sorted ccw
    let closest_node_by_angle = binary_search_angles(&poly.hull[1..], 0, poly.hull.len() - 2, &center, search_angle);


    let left = &poly.hull[closest_node_by_angle];
    let closest = &poly.hull[(closest_node_by_angle + 1) % poly.hull.len()];
    let right = &poly.hull[(closest_node_by_angle + 2) % poly.hull.len()];

    //println!("closest: {closest_node_by_angle} => {:02.3} \n\n", angle(&(closest - center)));

    Orientation::calc(left, p, closest) == Leftwards &&
        Orientation::calc(closest, p, right) == Leftwards
}

fn add_point(doc: Document, p: &Point, color: &str, radius: i32, stroke: &str) -> Document {
    let c = Circle::new()
        .set("cx", p.x)
        .set("cy", p.y)
        .set("fill", color)
        .set("stroke", stroke)
        .set("stroke-width", 2)
        .set("r", format!("{radius}"));
    doc.add(c)
}

fn grayscale_hex(p: f64) -> String {
    let s = ((p * 255.0) as i16).min(255).max(0);
    format!("#{:02x}{:02x}{:02x}", s, s, s)
}

fn add_text(doc: Document, p: &Point, text: String) -> Document {
    let c = Text::new()
        .set("x", p.x + 5.0)
        .set("y", p.y - 5.0)
        .set("fill", "white")
        .set("stroke", "black")
        .set("stroke-width", 0.8)
        .set("font-size", 18.0)
        .set("font-weight", "bold")
        .add(node::Text::new(text));
    doc.add(c)
}

const WIDTH: f64 = 500.0;
const HEIGHT: f64 = WIDTH;
const SPACING: f64 = 40.0;

const GREEN_FILL: &str = "#90ED90";
const GREEN_STROKE: &str = "#006300";
const RED_FILL: &str = "#ff0000";
const RED_STROKE: &str = "#8A0000";

const RED_OUTSIDE_FILL: &str = "#ff9999";
const RED_OUTSIDE_STROKE: &str = "#8A0000";

const POINT_RADIUS: i32 = 5;
const POINT_OUTSIDE_RADIUS: i32 = 4;

fn test_point_triangle() {
    let mut document = Document::new()
        .set("viewBox", (0, 0, WIDTH, HEIGHT))
        .set("width", WIDTH)
        .set("height", HEIGHT)
        .add(
            Rectangle::new()
                .set("fill", "white")
                .set("width", WIDTH)
                .set("height", HEIGHT)
        );

    let mut points = vec![];

    let dist = Uniform::new(SPACING, WIDTH - SPACING);
    for _ in 0..3 {
        let p = Point::new(thread_rng().sample(dist), thread_rng().sample(dist));
        points.push(p);
    }

    let poly = ConvexPoly::new(points.clone());

    let mut data = Data::new();
    let start = &poly.hull[0];
    data = data.move_to((start.x, start.y));
    for p in &poly.hull {
        data = data.line_to((p.x, p.y));
    }

    data = data.close();

    let path = Path::new()
        .set("fill", GREEN_FILL)
        .set("stroke", GREEN_STROKE)
        .set("stroke-width", 2)
        .set("d", data);

    document = document.add(path);

    for i in 0..poly.hull.len() {
        document = add_point(document, &poly.hull[i], GREEN_FILL, POINT_RADIUS, GREEN_STROKE);
    }

    let testpoint = Point::new(thread_rng().sample(dist), thread_rng().sample(dist));

    if poly.hull.len() < 3 {
        panic!("hull too small");
    }

    let mut inside = false;


    let A = &poly.hull[0];
    let B = &poly.hull[1];
    let C = &poly.hull[2];
    inside = Orientation::calc(A, &testpoint, B) == Leftwards &&
        Orientation::calc(B, &testpoint, C) == Leftwards &&
        Orientation::calc(C, &testpoint, A) == Leftwards;

    if inside {
        document = add_point(document, &testpoint, RED_FILL, POINT_RADIUS, RED_STROKE);
    } else {
        document = add_point(document, &testpoint, RED_OUTSIDE_FILL, POINT_OUTSIDE_RADIUS, RED_OUTSIDE_STROKE);
    }

    // average point
    let avg = (A + B + C) / 3.0;
    document = add_point(document, &avg, "#00ffff", POINT_RADIUS, "#004444");

    svg::save("triangle.svg", &document).unwrap();
}

fn test_point_polygon() {
    let mut document = Document::new()
        .set("viewBox", (0, 0, WIDTH, HEIGHT))
        .set("width", WIDTH)
        .set("height", HEIGHT)
        .add(
            Rectangle::new()
                .set("fill", "white")
                .set("width", WIDTH)
                .set("height", HEIGHT)
        );

    let mut points = vec![];

    let dist = Uniform::new(SPACING, WIDTH - SPACING);
    for _ in 0..40 {
        let p = Point::new(thread_rng().sample(dist), thread_rng().sample(dist));
        points.push(p);
    }

    println!("{:#?}", points);

    let poly = ConvexPoly::new(points.clone());

    let mut data = Data::new();
    let start = &poly.hull[0];
    data = data.move_to((start.x, start.y));
    for p in &poly.hull {
        data = data.line_to((p.x, p.y));
    }

    data = data.close();

    let path = Path::new()
        .set("fill", GREEN_FILL)
        .set("stroke", GREEN_STROKE)
        .set("stroke-width", 2)
        .set("d", data);

    document = document.add(path);

    let mut testpoint = Point::new(thread_rng().sample(dist), thread_rng().sample(dist));
    println!("testpoint: {:?}", testpoint);

    // triangulation lines
    let mut data = Data::new();
    let start = &poly.hull[0];
    for p in &poly.hull {
        data = data.move_to((start.x, start.y));
        data = data.line_to((p.x, p.y));
    }
    data = data.move_to((start.x, start.y));
    data = data.line_to((testpoint.x, testpoint.y));
    data = data.close();

    let path = Path::new()
        .set("fill", "none")
        .set("stroke", GREEN_STROKE)
        .set("stroke-width", 1)
        .set("d", data);

    document = document.add(path);

    for i in 0..poly.all.len() {
        document = add_point(document, &poly.all[i], GREEN_FILL, POINT_RADIUS, GREEN_STROKE);
    }

    let inside = is_point_in_polygon_fast(&poly, &testpoint);
    assert_eq!(is_point_in_polygon_fast(&poly, &testpoint), is_point_in_polygon(&poly, &testpoint));
    if inside {
        document = add_point(document, &testpoint, RED_FILL, POINT_RADIUS, RED_STROKE);
    } else {
        document = add_point(document, &testpoint, RED_OUTSIDE_FILL, POINT_OUTSIDE_RADIUS, RED_OUTSIDE_STROKE);
    }

    let center = poly.hull[0];
    let mut search_angle = angle(&(testpoint - center));
    if search_angle < 0.0 {
        println!("search angle negative");
        search_angle += 2.0 * f64::PI();
    }
    let closest_node_by_angle = binary_search_angles(&poly.hull[1..], 0, poly.hull.len() - 2, &center, search_angle);
    let closest = &poly.hull[closest_node_by_angle + 1];

    let c = Circle::new()
        .set("cx", closest.x)
        .set("cy", closest.y)
        .set("fill", "none")
        .set("stroke", "#00ffff")
        .set("stroke-width", 1)
        .set("stroke-dasharray", "2 1")
        .set("r", 11);
    document = document.add(c);

    for i in 0..poly.hull.len() {
        document = add_text(document, &poly.hull[i], format!("{i}"));
    }

    svg::save("polygon.svg", &document).unwrap();
}

fn test_red_points_green_triangles() {
    let mut document = Document::new()
        .set("viewBox", (0, 0, WIDTH, HEIGHT))
        .set("width", WIDTH)
        .set("height", HEIGHT)
        .add(
            Rectangle::new()
                .set("fill", "white")
                .set("width", WIDTH)
                .set("height", HEIGHT)
        );


    let mut green = vec![];
    let mut red = vec![];

    //let dist = Uniform::new(SPACING, WIDTH - SPACING);
    let dist = Normal::new(WIDTH / 2.0, WIDTH / 7.0).unwrap();
    let dist2 = Normal::new(WIDTH / 2.0, WIDTH / 6.0).unwrap();
    for _ in 0..25 {
        let g = Point::new(thread_rng().sample(dist), thread_rng().sample(dist));
        let r = Point::new(thread_rng().sample(dist2), thread_rng().sample(dist2));
        green.push(g);
        red.push(r);
    }

    let green_poly = ConvexPoly::new(green.clone());

    let mut data = Data::new();
    let start = &green_poly.hull[0];
    data = data.move_to((start.x, start.y));
    for p in &green_poly.hull {
        data = data.line_to((p.x, p.y));
    }
    data = data.close();

    let path = Path::new()
        .set("fill", GREEN_FILL)
        .set("stroke", GREEN_STROKE)
        .set("stroke-width", 2)
        .set("d", data);

    document = document.add(path);

    // triangulation lines
    let mut data = Data::new();
    let start = &green_poly.hull[0];
    for p in &green_poly.hull {
        data = data.move_to((start.x, start.y));
        data = data.line_to((p.x, p.y));
    }
    data = data.close();

    let path = Path::new()
        .set("fill", "none")
        .set("stroke", GREEN_STROKE)
        .set("stroke-width", 1)
        .set("d", data);

    document = document.add(path);

    for g in &green {
        document = add_point(document, g, GREEN_FILL, 5, GREEN_STROKE);
    }
    for r in &red {
        assert_eq!(is_point_in_polygon_fast(&green_poly, r), is_point_in_polygon(&green_poly, r));

        if is_point_in_polygon_fast(&green_poly, r) {
            document = add_point(document, r, RED_FILL, 5, RED_STROKE);
        } else {
            document = add_point(document, r, RED_OUTSIDE_FILL, 4, RED_OUTSIDE_STROKE);
        }
    }

    for i in 0..green_poly.hull.len() {
        document = add_text(document, &green_poly.hull[i], format!("{i}"));
    }

    svg::save("redgreen.svg", &document).unwrap();
}

fn main() {
    while true {
        test_point_triangle();
        test_point_polygon();
        test_red_points_green_triangles();
        thread::sleep(Duration::from_millis(2500));
    }
}
