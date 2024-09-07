use core::f64;
use std::{fmt::Display, ops::{Add, Div, Mul, Sub}};

use nannou::prelude::*;

// This code has been ported from here: https://observablehq.com/@rreusser/periodic-planar-three-body-orbits


struct Model {
    trajectory: Vec<State>,
    period: f64
}

impl Model {
    fn get_state(&self, time: f64) -> State
    {
        // Find nearest pair of states
        let offset = time % self.period;
        let mut min = 0;
        let mut max = self.trajectory.len()-1;
        while max-min > 1{
            let mid = (min + max) / 2;
            let time = self.trajectory[mid].time;
            if time < offset 
            {
                min = mid;
            } 
            else 
            {
                max = mid;
            }
        }
        
        let min_state = self.trajectory[min];
        let max_state = self.trajectory[min + 1 % self.trajectory.len()];
        // Calculate the midpoint between the two states based on the percentage traveled
        let trajectory_span = if max_state.time == 0.0 { self.period } else {max_state.time} - min_state.time;
        
        let current_span = offset - min_state.time;
        let percentage = current_span / trajectory_span;

        State {
            bodies: (min_state.bodies * (1.0 - percentage) + (max_state.bodies * percentage)),
            time: offset
        }
    }
}

fn main() {
    nannou::app(model)
        .simple_window(view)
        .run();
}

fn model(_app: &App) -> Model {
    let initial_conditions = InitialConditions {
        period: 6.234671,
        bodies: ThreeBodies {
            bodies: [
                Body {
                    position: Coordinate {
                        x: -1.0,
                        y: 0.0
                    },
                    velocity: Coordinate {
                        x: 0.306893,
                        y: 0.125507,
                    }
                },
                Body {
                    position: Coordinate {
                        x: 1.0,
                        y: 0.0
                    },
                    velocity: Coordinate {
                        x: 0.306893,
                        y: 0.125507,
                    }
                },
                Body {
                    position: Coordinate {
                        x: 0.0,
                        y: 0.0
                    },
                    velocity: Coordinate {
                        x: -0.613786,
                        y: -0.251014,
                    }
                },
            ],
            masses: [1.0, 1.0, 1.0]
        },
        author: "Šuvakov".to_string(),
        name: "Butterfly I.2.1".to_string()
    };

    let trajectory = calculate_trajectory(&initial_conditions);

    Model {
        trajectory,
        period: initial_conditions.period
    }
}

fn view(app: &App, model: &Model, frame: Frame) 
{
    let draw = app.draw();
    draw.background().color(PLUM);
    let speed = 0.0001;
    let state = model.get_state(app.duration.since_start.as_millis() as f64 * speed);
    let scale = 400.0;
    let x_offset = 0.0;
    let y_offset = 0.0;
    for i in 0..3 {
        let size = (state.bodies.masses[i] * 10.0) as f32;
        let position = state.bodies.bodies[i].position;
        let color = if i == 0 {GREEN} else if i == 1 {BLUE} else {YELLOW};
        draw.ellipse().w_h(size, size).x_y((position.x * scale + x_offset) as f32, (position.y * scale + y_offset) as f32).color(color);
    }

    let fps = app.fps().abs().floor() as u32;
    if fps < 60 {
        let title = format!("{}fps", fps);
        app.main_window().set_title( &title );
    }

    let _ = draw.to_frame(app, &frame);
}

#[derive(Clone, Copy, PartialEq)]
struct Coordinate
{
    x: f64,
    y: f64
}

impl Add for Coordinate {
    type Output = Coordinate;

    fn add(self, rhs: Self) -> Self::Output {
        Coordinate {
            x: self.x + rhs.x,
            y: self.y + rhs.y
        }
    }
}

impl Sub for Coordinate {
    type Output = Coordinate;

    fn sub(self, rhs: Self) -> Self::Output {
        Coordinate {
            x: self.x - rhs.x,
            y: self.y - rhs.y
        }
    }
}

impl Div<f64> for Coordinate
{
    type Output = Coordinate;

    fn div(self, rhs: f64) -> Self::Output {
        Coordinate {
            x: self.x / rhs,
            y: self.y / rhs
        }
    }
}

impl Mul<f64> for Coordinate
{
    type Output = Coordinate;

    fn mul(self, rhs: f64) -> Self::Output {
        Coordinate {
            x: self.x * rhs,
            y: self.y * rhs
        }
    }
}

impl Coordinate {
    fn error(self) -> f64 
    {
        self.x * self.x + self.y * self.y
    }

    fn distance(self, other: Coordinate) -> Coordinate
    {
        let offset = other - self;
        let r3 = f64::powf(offset.x * offset.x + offset.y * offset.y, 1.5);
        offset / r3
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Body {
    position: Coordinate,
    velocity: Coordinate
}

impl Add for Body
{
    type Output = Body;

    fn add(self, rhs: Self) -> Self::Output {
        Body {
            position: self.position + rhs.position,
            velocity: self.velocity + rhs.velocity,
        }
    }
}

impl Mul<f64> for Body
{
    type Output = Body;

    fn mul(self, rhs: f64) -> Self::Output {
        Body {
            position: self.position * rhs,
            velocity: self.velocity * rhs,
        }
    }
}

impl Body
{
    fn error(self) -> f64 
    {
        self.position.error() + self.velocity.error()   
    }
}

#[derive(Clone, Copy, PartialEq)]
struct ThreeBodies 
{
    bodies: [Body; 3],
    masses: [f64; 3]
}

impl Mul<f64> for ThreeBodies
{
    type Output = ThreeBodies;

    fn mul(self, rhs: f64) -> Self::Output {
        ThreeBodies
        {
            bodies: self.bodies.map(|f| f * rhs),
            masses: self.masses
        }
    }
}

impl ThreeBodies
{
    fn error(self) -> f64 
    {
        self.bodies[0].error() + self.bodies[1].error() + self.bodies[2].error()
    }
}


impl Add for ThreeBodies
{
    type Output = ThreeBodies;

    fn add(self, rhs: Self) -> Self::Output {
        ThreeBodies
        {
            bodies: [self.bodies[0] + rhs.bodies[0], self.bodies[1] + rhs.bodies[1], self.bodies[2] + rhs.bodies[2]],
            masses: self.masses
        }
    }
}
impl Display for ThreeBodies
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.bodies[0].position.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[0].position.y.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[0].velocity.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[0].velocity.y.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[1].position.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[1].position.y.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[1].velocity.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[1].velocity.y.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[2].position.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[2].position.y.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[2].velocity.x.fmt(f)?;
        f.write_str(", ")?;
        self.bodies[2].velocity.y.fmt(f)?;
        Ok(())
    }
}


#[derive(Clone)]
struct InitialConditions 
{
    period: f64,
    bodies: ThreeBodies,
    name: String,
    author: String
}

#[derive(Clone, Copy)]
struct State 
{
    bodies: ThreeBodies,
    time: f64,
}

#[derive(Clone, Copy)]
struct Step 
{
    state: State,
    dt: f64,
    limit_reached: bool
}


fn planar_three_body_derivative(mut state: ThreeBodies) -> ThreeBodies
{
    let distance01 = state.bodies[0].position.distance(state.bodies[1].position);
    let distance02 = state.bodies[0].position.distance(state.bodies[2].position);
    let distance12 = state.bodies[1].position.distance(state.bodies[2].position);

    state.bodies[0].position = state.bodies[0].velocity;
    state.bodies[1].position = state.bodies[1].velocity;
    state.bodies[2].position = state.bodies[2].velocity;

    // Calculate attractions
    state.bodies[0].velocity = distance01 * state.masses[1] + distance02 * state.masses[2];
    state.bodies[1].velocity = distance01 * -state.masses[0] + distance12 * state.masses[2];
    state.bodies[2].velocity = distance02 * -state.masses[0] + distance12 * -state.masses[1];

    state
}

// Return the float with the smaller magnitude
fn min_mag(a: f64, b: f64) -> f64
{
    if a > 0.0
    {
        a.min(b)
    } 
    else 
    {
        a.max(b)
    }
}

// Return the float with the larger magnitude
fn max_mag(a: f64, b: f64) -> f64
{
    if a > 0.0
    {
        a.max(b)
    } 
    else 
    {
        a.min(b)
    }
}

fn calculate_trajectory(initial_conditions: &InitialConditions) -> Vec<State>
{
    let mut step = Step {
        state: State {
            bodies: initial_conditions.bodies,
            time: 0.0
        },
        dt: 1.0,
        limit_reached: false
    };
    let mut trajectory = vec![step.state];
    while !step.limit_reached
    {
        step = ode45(step, initial_conditions.period);
        trajectory.push(step.state)
    }
    
    return trajectory
}

fn ode45(step: Step, period: f64) -> Step
{
    let tolerance = 1e-9;
    let tolerance2 = tolerance * tolerance;
    let t_limit = period;
    let safety_factor = 0.9;
    let y = step.state.bodies;
    let mut dt = step.dt;
    let k1 = planar_three_body_derivative(y);
    for _ in 0..1000
    {
        dt = min_mag(dt, t_limit - step.state.time);
        let mut w: ThreeBodies = y + k1 * (0.2 * dt);
        let k2 = planar_three_body_derivative(w);
        w = y + (k1 * 0.075 + k2 * 0.225) * dt;
        let k3 = planar_three_body_derivative(w);
        w = y + (k1 * 0.3 + k2 * -0.9 + k3 * 1.2) * dt;
        let k4 = planar_three_body_derivative(w);
        w = y + (k1 * -0.203703703703703703 + k2 * 2.5 + k3 * -2.592592592592592592 + k4 * 1.296296296296296296) * dt;
        let k5 = planar_three_body_derivative(w);
        w = y + (k1 * 0.029495804398148148 + k2 * 0.341796875 + k3 * 0.041594328703703703 + k4 * 0.400345413773148148 + k5 * 0.061767578125) * dt;
        let k6 = planar_three_body_derivative(w);
        let error_body = (k1 * 0.004293774801587301 + k3 * -0.018668586093857832 + k4 * 0.034155026830808080 + k5 * 0.019321986607142857 + k6 * -0.039102202145680406) * dt;
        let error2 = error_body.error();

        if error2 < tolerance2 || dt == 0.0
        {
            let result = y + (k1 * 0.097883597883597883 + k3 * 0.402576489533011272 + k4 * 0.210437710437710437 + k6 * 0.289102202145680406) * dt;
            let next_t = step.state.time + dt;
            let next_dt = safety_factor * dt * f64::powf(tolerance2 / error2, 0.125);
            let out_dt = max_mag(dt / 10.0, min_mag(dt * 10.0, next_dt));
            let limit_reached = t_limit.is_finite() && ((next_t - t_limit) / dt).abs() < f64::EPSILON;
            
            return Step {
                state: State {
                    bodies: result,
                    time: next_t
                },
                dt: out_dt,
                limit_reached,
            }
        }

        let next_dt = safety_factor * dt * f64::powf(tolerance2 / error2, 0.1);
        dt = max_mag(dt / 10.0, next_dt)
    }

    panic!("Unable to calculate limit in time!");
}