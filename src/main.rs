use macroquad::prelude::*;
use rayon::prelude::*;

const RADIUS: f32 = 3.0;
// const CONSTRAINT_RADIUS: f32 = 300.0;
// const SUBSTEPS: u32 = 8;

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

    pub fn update(&mut self, objects: &mut [VerletObject], dt: f32, substeps: u32) {
        let sub_dt = dt / substeps as f32;
        for _ in 0..substeps {
            Self::apply_gravity(objects, &self.gravity);
            Self::apply_constraints(objects);
            Self::solve_collisions(objects);
            Self::update_positions(objects, sub_dt);
        }
    }

    fn apply_gravity(objects: &mut [VerletObject], gravity: &Vec2) {
        for object in objects.iter_mut() {
            object.accelerate(*gravity);
        }
    }

    fn update_positions(objects: &mut [VerletObject], dt: f32) {
        objects.par_iter_mut().for_each(|object| {
            object.update_position(dt);
        });
    }

    fn apply_constraints(objects: &mut [VerletObject]) {
        let screen_width = screen_width();
        let screen_height = screen_height();
        let position: Vec2 = Vec2::new(screen_width / 2.0, screen_height / 2.0);
        for object in objects.iter_mut() {
            let to_obj = object.get_position() - position;
            let distance = to_obj.length();
            if distance > screen_width / 4.0 - RADIUS {
                let n = to_obj / distance; // TODO: maybe normalize?
                object.position_current = position + n * (screen_width / 4.0 - RADIUS);
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

fn convert_velocity_to_color(velocity: Vec2) -> Color {
    // slow - blue
    // medium - green
    // fast - red
    // so this is hue shift from blue to red

    let speed = velocity.length();
    let max_speed = 5.0;

    // clamp speed to [0, max_speed]
    let clamped_speed = speed.min(max_speed);

    // map speed to [0, 1]
    let normalized_speed = clamped_speed / max_speed;

    // map speed to hue in [240, 0] (blue to red in HSL color space)
    let hue = 240.0 * (1.0 - normalized_speed);

    // convert HSL to RGB
    let (r, g, b) = hsl_to_rgb(hue, 1.0, 0.5);

    Color::new(r, g, b, 1.0)
}

fn hsl_to_rgb(h: f32, s: f32, l: f32) -> (f32, f32, f32) {
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = l - c / 2.0;

    let (rp, gp, bp) = if h < 60.0 {
        (c, x, 0.0)
    } else if h < 120.0 {
        (x, c, 0.0)
    } else if h < 180.0 {
        (0.0, c, x)
    } else if h < 240.0 {
        (0.0, x, c)
    } else if h < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };

    (rp + m, gp + m, bp + m)
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

    let mut substeps = 8;

    loop {
        // Clear the screen
        clear_background(BLACK);

        let screen_width = screen_width();
        let screen_height = screen_height();

        // If the space is pressed, clear the points
        if is_key_pressed(KeyCode::Space) {
            objects.clear();
        }

        // Change the number of substeps
        let (_, mouse_wheel_y) = mouse_wheel();
        if mouse_wheel_y > 0.0 {
            substeps = (substeps + 1).min(32);
        } else if mouse_wheel_y < 0.0 {
            substeps = (substeps - 1).max(1);
        }

        // Setup the center of the constraint circle
        let constraint_center = Vec2::new(screen_width / 2.0, screen_height / 2.0);

        let fps = (1.0 / get_frame_time()).round();

        // Top left text
        draw_text(&format!("FPS: {}", fps), 10.0, 20.0, 20.0, WHITE);
        draw_text(
            &format!("Objects: {}", objects.len()),
            10.0,
            40.0,
            20.0,
            WHITE,
        );
        draw_text(&format!("Substeps: {}", substeps), 10.0, 60.0, 20.0, WHITE);

        // Top right text
        draw_text("CLICK TO ADD POINT", screen_width - 165., 20.0, 20.0, WHITE);
        draw_text("SPACE TO CLEAR", screen_width - 132., 40.0, 20.0, WHITE);
        draw_text(
            "SCROLL TO CHANGE SUBSTEPS",
            screen_width - 228.,
            60.0,
            20.0,
            WHITE,
        );

        // Add a point
        if is_mouse_button_down(MouseButton::Left) {
            let current_time = get_time();
            if current_time - last_mouse_input > 0.01 {
                last_mouse_input = current_time;
                let mouse_position = mouse_position();
                objects.push(VerletObject::new(Vec2::new(
                    mouse_position.0,
                    mouse_position.1,
                )));
            }
        }

        // Update the solver
        solver.update(&mut objects, get_frame_time(), substeps);

        // Draw the constraint circle
        draw_poly_lines(
            constraint_center.x,
            constraint_center.y,
            100,
            screen_width / 4.0,
            0.,
            1.,
            WHITE,
        );

        // Draw the points
        for object in objects.iter() {
            draw_circle(
                object.get_position().x,
                object.get_position().y,
                RADIUS,
                convert_velocity_to_color(object.get_position() - object.position_old),
            );
        }

        // info!("First point pos: {:?}", objects[0].get_position());
        // info!("Len: {}", objects.len());

        // Finish the frame
        next_frame().await
    }
}
