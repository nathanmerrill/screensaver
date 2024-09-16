use std::{error::Error, fs::File, io::BufReader, ops::{Add, Mul}};

use nannou::{prelude::*, rand::{self, Rng}};
use serde::Deserialize;

// This code has been ported from here: https://observablehq.com/@rreusser/periodic-planar-three-body-orbits

struct Model {
    trajectory: Vec<State>,
    trajectory_index: usize,
    inverted: bool,
    trails: Vec<[(Vec2, Vec2); 3]>,
    bodies: [Vec2; 3],
    period: f32,
    boundaries: Vec2,
    scale: f32,
    colors: [Hsl; 3],
    title: String,
}

impl Model {
    fn update_bodies(&mut self, time: f32)
    {
        // Find nearest pair of states
        let offset = time % self.period;
        
        let mut min_state ;
        let mut max_state ;
        loop
        {
            min_state = self.trajectory[self.trajectory_index];
            let next_index = (self.trajectory_index + 1) % self.trajectory.len();
            max_state = self.trajectory[next_index];
            if min_state.time <= offset && max_state.time >= offset {
                break;
            }

            self.trajectory_index = next_index;
        }
        
        // Calculate the midpoint between the two states based on the percentage traveled
        let trajectory_span = (max_state.time - min_state.time).rem_euclid(self.period);
        
        let current_span = offset - min_state.time;
        let percentage = current_span / trajectory_span;

        // Mutate max_state bodies, as it is difficult to iterate over both arrays at once and then collect into a fixed-size array
        min_state.bodies.into_iter()
            .zip(max_state.bodies.iter_mut())
            .for_each(|(min, max)| *max = min.lerp(*max, percentage));

        self.bodies = max_state.bodies.map(|f| f * self.scale)
    }

    fn update_path(&mut self) 
    {
        if self.trajectory_index == 0 {
            if self.trails.len() > 1 {
                if !self.inverted {
                    self.inverted = true;
                    for path in self.trails.iter_mut() {
                        path.reverse()
                    }
                }
            } else {
                self.inverted = false
            }
        }

        if self.inverted {
            while self.trails.len() > self.trajectory.len() - self.trajectory_index - 4 {
                self.trails.pop();
            }
        }
        else {
            while self.trails.len() < self.trajectory_index {
                self.trails.push([0,1,2].map(|index| (self.trajectory[self.trails.len()].bodies[index]*self.scale, self.trajectory[self.trails.len()+1].bodies[index]*self.scale)));
            }
        }
    }

    fn randomize_colors(&mut self)
    {
        let mut rng = rand::thread_rng();
    
        let color_diff = 15.0;
    
        let a = rng.gen::<f32>() * 360.0;
        let b = (a + rng.gen::<f32>() * (360.0 - color_diff*2.0) + color_diff) % 360.0;
        
        let mut c = rng.gen::<f32>() * (360.0 - color_diff*4.0);
        if c > (a - color_diff) {
            c += color_diff * 2.0;
            if c > (b - color_diff) {
                c += color_diff * 2.0;
            }
        } 
        else if c > (b - color_diff) 
        {
            c += color_diff * 2.0;
            if c > (a - color_diff) {
                c += color_diff * 2.0;
            }
        }
    
        let color_a = Hsl::new(a, 1.0, 0.5);
        let color_b = Hsl::new(b, 1.0, 0.5);
        let color_c = Hsl::new(c, 1.0, 0.5);
        
        self.colors = [color_a, color_b, color_c]
    }
    
    fn reset(&mut self) {

    }

    fn init(initial_conditions: &InitialConditions) -> Self {
        let mut trajectory = calculate_trajectory(&initial_conditions);

        let positions = trajectory.iter().flat_map(|s| s.bodies);
        let min_position = positions.clone().reduce(|a, b| a.min(b)).unwrap();
        let max_position = positions.reduce(|a, b| a.max(b)).unwrap();
    
        let wh = max_position-min_position;
        
        let offset = (wh / -2.0) - min_position;

        for trajectory in trajectory.iter_mut() {
            for body in trajectory.bodies.iter_mut() {
                *body += offset;
            }
        }

        let mut distance_sum = 0.0;

        for (state1, state2) in trajectory.iter().zip(trajectory.iter().skip(1).chain(trajectory.first()))
        {
            let max_distance = state1.bodies.iter().zip(state2.bodies).map(|(body1, body2)| body2.distance(*body1)).max_by(f32::total_cmp).unwrap();
            distance_sum += max_distance;            
        }

        let average_speed = distance_sum / initial_conditions.period as f32;

        println!("Average speed: {:?}", average_speed);
    
        let mut model = Model {
            trajectory,
            trajectory_index: 0,
            period: initial_conditions.period as f32,
            boundaries: wh,
            trails: vec![],
            scale: 1.0,
            inverted: false,
            bodies: [Vec2::ZERO, Vec2::ZERO, Vec2::ZERO],
            title: format!("{}: {}", initial_conditions.author, initial_conditions.name),
            colors: [Hsl::default(), Hsl::default(), Hsl::default()]
        };

        model.randomize_colors();

        model
    }

    fn update_scale(&mut self, dim: Vec2) 
    {
        let old_scale = self.scale;
        let drawable_area = dim * 0.90;
        let scale = drawable_area / self.boundaries;
        self.scale = scale.min_element();
        
        let scale_change = self.scale / old_scale;

        for trail in self.trails.iter_mut() {
            for (item1, item2) in trail.iter_mut() {
                *item1 *= scale_change;
                *item2 *= scale_change;
            }
        }

        self.bodies = self.bodies.map(|f| f * scale_change)
    }
}

fn main() {
    nannou::app(model)
        .event(event)
        .update(update)
        .simple_window(view)
        .run();
}

fn event(app: &App, model: &mut Model, event: Event) {
    match event {
        Event::WindowEvent { id: _, simple: Some(window_event) } => {
            match window_event {
                WindowEvent::Resized(dim) => {
                    model.update_scale(dim);
                },
                WindowEvent::KeyReleased(Key::Left) | WindowEvent::KeyReleased(Key::Right) => {
                    model.reset();
                }
                WindowEvent::KeyPressed(_) | WindowEvent::MousePressed(_) => {
                    app.quit();
                }
                _ => {}                
            }
        }
        _ => {}
    }
}

fn update(_app: &App, model: &mut Model, update: Update) 
{
    let time = update.since_start.as_secs_f32() * 0.1;
    model.update_bodies(time);
    model.update_path();
}

fn deserialize(path: &str) -> Result<Vec<InitialConditions>, Box<dyn Error>> {
    // Open the file in read-only mode with buffer.
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    // Read the JSON contents of the file as an instance of `User`.
    let u = ron::de::from_reader(reader)?;

    // Return the `User`.
    Ok(u)
}

fn model(app: &App) -> Model {
    let path = "initialconditions.ron";
    let initial_conditions = deserialize(path).unwrap();

    let mut model = Model::init(&initial_conditions.last().unwrap());
    model.update_scale(app.main_window().rect().wh());
    model
}

fn view(app: &App, model: &Model, frame: Frame) 
{
    let draw = app.draw();
    draw.background().color(BLACK);

    let text_height = 20.0;

    draw.text(&model.title).h(text_height).w(app.main_window().rect().w()).y((app.window_rect().h() - text_height)/-2.0);

    for item in &model.trails {
        for (index, (start, end)) in item.iter().enumerate() {
            draw.line().start(*start).end(*end).color(model.colors[index]);
        }
    }

    let endpoints = if model.inverted { // Fetch last point in line
        model.trails.first()
        .map(|a| a.map(|b| b.0))
    } else {
        model.trails.last()
        .map(|a| a.map(|b| b.1))
    }.unwrap_or_else(||model.trajectory[0].bodies.map(|f| f*model.scale)); // If the line is empty, fetch the starting trajectory point

    for (index, endpoint) in endpoints.iter().enumerate() {
        draw.line().start(*endpoint).end(model.bodies[index]).color(model.colors[index]);
    }

    for i in 0..3 {
        let color = model.colors[i];
        let position = model.bodies[i];
        draw.ellipse().w_h(10.0, 10.0).xy(position).color(color);
    }

    let _ = draw.to_frame(app, &frame);
}

#[derive(Clone, Copy, PartialEq, Deserialize)]
struct ThreeBodies 
{
    bodies: [DMat2; 3],
    #[serde(default = "default_masses")] 
    masses: [f64; 3]
}

fn default_masses() -> [f64; 3] {
    [1.0, 1.0, 1.0]
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
        self.bodies.iter().flat_map(|f| [f.col(0), f.col(1)]).map(|f| f.length_squared()).sum()
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

#[derive(Clone, Deserialize)]
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
    bodies: [Vec2; 3],
    time: f32,
}

#[derive(Clone, Copy)]
struct Step 
{
    bodies: ThreeBodies,
    time: f64,
    dt: f64,
    limit_reached: bool
}

impl Step {
    fn state(&self) -> State {
        State {
            bodies: self.bodies.bodies.map(|f| f.col(0).as_f32()),
            time: self.time as f32
        }
    }
}

fn distance(position1: DVec2, position2: DVec2) -> DVec2
{    
    let offset = position2 - position1;
    let r3 = f64::powf(offset.length_squared(), 1.5);
    offset / r3
}


fn planar_three_body_derivative(mut state: ThreeBodies) -> ThreeBodies
{
    let distance01 = distance(state.bodies[0].col(0), state.bodies[1].col(0));
    let distance02 = distance(state.bodies[0].col(0), state.bodies[2].col(0));
    let distance12 = distance(state.bodies[1].col(0), state.bodies[2].col(0));

    *state.bodies[0].col_mut(0) = state.bodies[0].col(1);
    *state.bodies[1].col_mut(0) = state.bodies[1].col(1);
    *state.bodies[2].col_mut(0) = state.bodies[2].col(1);

    // Calculate attractions
    *state.bodies[0].col_mut(1) = distance01 * state.masses[1] + distance02 * state.masses[2];
    *state.bodies[1].col_mut(1) = distance01 * -state.masses[0] + distance12 * state.masses[2];
    *state.bodies[2].col_mut(1) = distance02 * -state.masses[0] + distance12 * -state.masses[1];

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
        bodies: initial_conditions.bodies,
        time: 0.0,
        dt: 1.0,
        limit_reached: false
    };
    let mut trajectory = vec![step.state()];
    while !step.limit_reached
    {
        step = ode45(step, initial_conditions.period);
        trajectory.push(step.state())
    }
    
    return trajectory
}

fn ode45(step: Step, period: f64) -> Step
{
    let tolerance = 1e-9;
    let tolerance2 = tolerance * tolerance;
    let t_limit = period;
    let safety_factor = 0.9;
    let y = step.bodies;
    let mut dt = step.dt;
    let k1 = planar_three_body_derivative(y);
    for _ in 0..1000
    {
        dt = min_mag(dt, t_limit - step.time);
        let mut w: ThreeBodies = y + k1 * (0.2 * dt);
        let k2 = planar_three_body_derivative(w);
        w = y + (k1 * 0.075 + k2 * 0.225) * dt;
        let k3 = planar_three_body_derivative(w);
        w = y + (k1 * 0.3 + k2 * -0.9 + k3 * 1.2) * dt;
        let k4 = planar_three_body_derivative(w);
        w = y + (k1 * (-11.0/54.0) + k2 * 2.5 + k3 * (-70.0/27.0) + k4 * (35.0/27.0)) * dt;
        let k5 = planar_three_body_derivative(w);
        w = y + (k1 * (1631.0/55296.0) + k2 * (175.0/512.0) + k3 * (575.0/13824.0) + k4 * (44275.0/110592.0) + k5 * (253.0/4096.0)) * dt;
        let k6 = planar_three_body_derivative(w);
        let error_body = (k1 * (2825.0/27648.0 - 37.0/378.0) + k3 * (18575.0/48384.0 - 250.0/621.0) + k4 * (13525.0/55296.0 - 125.0/594.0) + k5 * (277.0/14336.0) + k6 * (0.25 - 512.0/1771.0)) * dt;
        let error2 = error_body.error();

        if error2 < tolerance2 || dt == 0.0
        {
            let result = y + (k1 * (37.0/378.0) + k3 * (250.0/621.0) + k4 * (125.0/594.0) + k6 * (512.0/1771.0)) * dt;
            let next_t = step.time + dt;
            let next_dt = safety_factor * dt * f64::powf(tolerance2 / error2, 0.125);
            let out_dt = max_mag(dt / 10.0, min_mag(dt * 10.0, next_dt));
            let limit_reached = t_limit.is_finite() && ((next_t - t_limit) / dt).abs() < f64::EPSILON;
            
            return Step {
                bodies: result,
                time: next_t,
                dt: out_dt,
                limit_reached,
            }
        }

        let next_dt = safety_factor * dt * f64::powf(tolerance2 / error2, 0.1);
        dt = max_mag(dt / 10.0, next_dt)
    }

    panic!("Unable to calculate limit in time!");
}