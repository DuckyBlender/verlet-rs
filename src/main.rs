use macroquad::prelude::*;
// use rayon::prelude::*;

const RADIUS: f32 = 10.0;
const CONSTRAINT_RADIUS: f32 = 250.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct VerletObject {
    position_current: Vec2,
    position_old: Vec2,
    acceleration: Vec2,
}

impl VerletObject {
    pub fn new(position: Vec2) -> Self {
        VerletObject {
            position_current: position,
            position_old: position,
            acceleration: Vec2::new(0., 0.),
        }
    }

    pub fn update_position(&mut self, dt: f32) {
        let velocity = self.position_current - self.position_old;
        // Save current position
        self.position_old = self.position_current;
        // Perform verlet integration
        self.position_current += velocity + self.acceleration * dt * dt;
        // Reset acceleration
        self.acceleration = Vec2::new(0., 0.);
    }

    pub fn accelerate(&mut self, acceleration: Vec2) {
        self.acceleration += acceleration;
    }

    pub fn get_position(&self) -> Vec2 {
        self.position_current
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Solver {
    gravity: Vec2,
}

impl Solver {
    pub fn new() -> Self {
        Solver {
            gravity: Vec2::new(0.0, 1000.0),
        }
    }

    pub fn update(&mut self, objects: &mut [VerletObject], dt: f32) {
        Self::apply_gravity(objects, &self.gravity);
        Self::apply_constraints(objects);
        Self::update_positions(objects, dt);
    }

    fn apply_gravity(objects: &mut [VerletObject], gravity: &Vec2) {
        for object in objects.iter_mut() {
            object.accelerate(*gravity);
        }
    }

    fn update_positions(objects: &mut [VerletObject], dt: f32) {
        for object in objects.iter_mut() {
            object.update_position(dt);
        }
    }

    fn apply_constraints(objects: &mut [VerletObject]) {
        let position: Vec2 = Vec2::new(400.0, 300.0);
        for object in objects.iter_mut() {
            let to_obj = object.get_position() - position;
            let distance = to_obj.length();
            if distance > CONSTRAINT_RADIUS - RADIUS {
                let n = to_obj / distance; // TODO: maybe normalize?
                object.position_current = position + n * (CONSTRAINT_RADIUS - RADIUS);
            }
        }
    }
}

#[macroquad::main("BasicShapes")]
async fn main() {
    // Setup a point in the middle of the screen
    let mut objects = vec![VerletObject::new(Vec2::new(
        screen_width() / 2.0,
        screen_height() / 2.0,
    ))];

    let mut solver = Solver::new();

    loop {
        // Clear the screen
        clear_background(BLACK);

        // Update the solver
        solver.update(&mut objects, get_frame_time());

        // Draw the constraint circle
        draw_poly_lines(400., 300., 100, CONSTRAINT_RADIUS, 0., 1., WHITE);

        // Draw the point
        draw_circle(
            objects[0].get_position().x,
            objects[0].get_position().y,
            RADIUS,
            RED,
        );

        info!("Point pos: {:?}", objects[0].get_position());

        // Finish the frame
        next_frame().await
    }
}
