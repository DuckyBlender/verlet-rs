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
        Self::solve_collisions(objects);
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

    fn solve_collisions(objects: &mut [VerletObject]) {
        // Brute force O(n^2) collision detection
        let object_count = objects.len();
        for i in 0..object_count {
            for j in i + 1..object_count {
                let collision_axis = objects[i].get_position() - objects[j].get_position();
                let distance: f32 = collision_axis.length();
                if distance < 2.0 * RADIUS {
                    // Collision detected
                    let n = collision_axis / distance;
                    let delta: f32 = 2.0 * RADIUS - distance;
                    objects[i].position_current += 0.5 * delta * n;
                    objects[j].position_current -= 0.5 * delta * n;
                }
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
    let mut last_mouse_input: f64 = 0.0;

    loop {
        // Clear the screen
        clear_background(BLACK);

        let fps = (1.0 / get_frame_time()).round();
        // Draw the FPS
        draw_text(&format!("FPS: {}", fps), 10.0, 20.0, 20.0, WHITE);

        // Add a point
        if is_mouse_button_down(MouseButton::Left) {
            let current_time = get_time();
            if current_time - last_mouse_input > 0.1 {
                last_mouse_input = current_time;
                let mouse_position = mouse_position();

                objects.push(VerletObject::new(Vec2::new(
                    mouse_position.0,
                    mouse_position.1,
                )));
            }
        }

        // Update the solver
        solver.update(&mut objects, get_frame_time());

        // Draw the constraint circle
        draw_poly_lines(400., 300., 100, CONSTRAINT_RADIUS, 0., 1., WHITE);

        // Draw the points
        for object in objects.iter() {
            draw_circle(
                object.get_position().x,
                object.get_position().y,
                RADIUS,
                WHITE,
            );
        }

        // info!("First point pos: {:?}", objects[0].get_position());
        info!("Len: {}", objects.len());

        // Finish the frame
        next_frame().await
    }
}
