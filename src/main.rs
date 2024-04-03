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
pub struct DebugTimeInfo {
    pub gravity_time: f32,
    pub constraints_time: f32,
    pub collisions_time: f32,
    pub update_positions_time: f32,
}

#[derive(Debug, Default)]
pub struct Solver {
    gravity: Vec2,
    // grid: Vec<Vec<Vec<usize>>>, // 1d 2d vec of point IDs
}

impl Solver {
    pub fn new() -> Self {
        // seperate the grid into RADIUS * 2 x RADIUS * 2 squares
        Solver {
            gravity: Vec2::new(0.0, 1000.0),
            // grid: vec![vec![vec![]]],
        }
    }

    pub fn update(
        &mut self,
        objects: &mut [VerletObject],
        dt: f32,
        substeps: u32,
    ) -> DebugTimeInfo {
        let sub_dt = dt / substeps as f32;
        let mut gravity_time = 0.0;
        let mut constraints_time = 0.0;
        let mut collisions_time = 0.0;
        let mut update_positions_time = 0.0;
        for _ in 0..substeps {
            gravity_time += Self::apply_gravity(objects, &self.gravity);
            constraints_time += Self::apply_constraints(objects);
            collisions_time += Self::solve_collisions(objects);
            update_positions_time += Self::update_positions(objects, sub_dt);
        }
        DebugTimeInfo {
            gravity_time,
            constraints_time,
            collisions_time,
            update_positions_time,
        }
    }

    // fn group_into_squares(&mut self, objects: &mut [VerletObject]) {
    //     let screen_width = screen_width();
    //     let screen_height = screen_height();
    //     // The grid is RADIUS * 2 x RADIUS * 2 px
    //     let grid_width = RADIUS * 2.0;
    //     }

    fn apply_gravity(objects: &mut [VerletObject], gravity: &Vec2) -> f32 {
        let now = std::time::Instant::now();
        for object in objects.iter_mut() {
            object.accelerate(*gravity);
        }
        now.elapsed().as_secs_f32()
    }

    fn update_positions(objects: &mut [VerletObject], dt: f32) -> f32 {
        let now = std::time::Instant::now();
        objects.par_iter_mut().for_each(|object| {
            object.update_position(dt);
        });
        now.elapsed().as_secs_f32()
    }

    fn apply_constraints(objects: &mut [VerletObject]) -> f32 {
        let now = std::time::Instant::now();
        let screen_width = screen_width();
        let screen_height = screen_height();
        for object in objects.iter_mut() {
            // TODO: if the object is above, dont check below
            if object.get_position().x < 0.0 + RADIUS + 1.0 {
                // radius and 1 for border
                object.position_current.x = 0.0 + RADIUS + 1.0;
            }
            if object.get_position().x > screen_width - RADIUS  - 1.0 {
                object.position_current.x = screen_width - RADIUS - 1.0;
            }
            if object.get_position().y < 0.0 + RADIUS + 1.0 {
                object.position_current.y = 0.0 + RADIUS  + 1.0;
            }
            if object.get_position().y > screen_height - RADIUS - 1.0 {
                object.position_current.y = screen_height - RADIUS - 1.0;
            }
        }
        now.elapsed().as_secs_f32()
    }

    fn solve_collisions(objects: &mut [VerletObject]) -> f32 {
        // returns time in seconds
        // Brute force O(n^2) collision detection
        let now = std::time::Instant::now();
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

        now.elapsed().as_secs_f32()
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

        let fps = (1.0 / get_frame_time()).round();

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
        let timings = solver.update(&mut objects, get_frame_time(), substeps);

        // Draw the constraint (entire screen)
        draw_rectangle_lines(0.0, 0.0, screen_width, screen_height, 2.0, WHITE);

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

        // Draw the timings in the bottom left
        draw_text(
            &format!("Gravity: {:.2}ms", timings.gravity_time * 1000.0),
            10.0,
            screen_height - 80.0,
            20.0,
            WHITE,
        );
        draw_text(
            &format!("Constraints: {:.2}ms", timings.constraints_time * 1000.0),
            10.0,
            screen_height - 60.0,
            20.0,
            WHITE,
        );
        draw_text(
            &format!("Collisions: {:.2}ms", timings.collisions_time * 1000.0),
            10.0,
            screen_height - 40.0,
            20.0,
            WHITE,
        );
        draw_text(
            &format!(
                "Update Positions: {:.2}ms",
                timings.update_positions_time * 1000.0
            ),
            10.0,
            screen_height - 20.0,
            20.0,
            WHITE,
        );

        // Finish the frame
        next_frame().await
    }
}
