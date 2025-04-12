// Entry point - Handles simulation loop and init
mod physics;

use ggez::{
    event,
    graphics::{self, Color, DrawMode, DrawParam, Mesh},
    Context, GameResult,
    input::mouse::MouseButton,
};
use nalgebra::Point2;
use physics::bodies::{Body, Material, BodyType, Shape};

const WINDOW_WIDTH: f32 = 800.0;
const WINDOW_HEIGHT: f32 = 600.0;
const BALL_RADIUS: f32 = 20.0;

// Define colors
const STATIC_COLOR: Color = Color::new(0.5, 0.5, 0.5, 1.0);  // Gray
const WOOD_COLOR: Color = Color::new(0.545, 0.271, 0.075, 1.0);  // Brown

struct MainState {
    world: physics::World,
}

impl MainState {
    fn new() -> GameResult<MainState> {
        let mut world = physics::World::new();
        
        // Add ground
        let ground = Body::new_rectangle(
            Point2::new(WINDOW_WIDTH as f64 / 2.0, WINDOW_HEIGHT as f64 - 20.0),
            WINDOW_WIDTH as f64,
            40.0,
            Material::stone(),
            BodyType::Static,
        );
        world.add_body(ground);

        // Add some blocks
        for i in 0..3 {
            let block = Body::new_rectangle(
                Point2::new(600.0 + (i as f64) * 50.0, WINDOW_HEIGHT as f64 - 90.0),
                40.0,
                80.0,
                Material::wood(),
                BodyType::Dynamic,
            );
            world.add_body(block);
        }

        Ok(MainState { world })
    }
}

impl event::EventHandler<ggez::GameError> for MainState {
    fn update(&mut self, _ctx: &mut Context) -> GameResult {
        // Update physics with a fixed time step
        self.world.update(1.0 / 60.0);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(ctx, Color::BLACK);

        // Draw each body
        for body in &self.world.bodies {
            let physics_x = body.position.x as f32;
            let physics_y = body.position.y as f32;

            // Convert physics Y (upwards) to screen Y (downwards)
            // Note: Ensure WINDOW_HEIGHT is accessible or pass it appropriately.
            // Assuming WINDOW_HEIGHT is a constant defined in this file.
            let screen_y = WINDOW_HEIGHT - physics_y; 

            match &body.shape {
                Shape::Circle { radius } => {
                    let r = *radius as f32;
                    let circle = Mesh::new_circle(
                        ctx,
                        DrawMode::fill(),
                        [physics_x, screen_y], // Use converted screen_y
                        r,
                        0.1, // Tolerance
                        match body.body_type {
                            BodyType::Static => STATIC_COLOR,
                            BodyType::Dynamic => Color::WHITE, // Simple white for dynamic circles
                        },
                    )?;
                    canvas.draw(&circle, DrawParam::default());
                }
                Shape::Rectangle { width, height } => {
                    let w = *width as f32;
                    let h = *height as f32;
                    let rect = Mesh::new_rectangle(
                        ctx,
                        DrawMode::fill(),
                        graphics::Rect::new(
                            physics_x - w / 2.0,
                            screen_y - h / 2.0, // Use converted screen_y
                            w,
                            h,
                        ),
                        match body.body_type {
                            BodyType::Static => STATIC_COLOR,
                            BodyType::Dynamic => WOOD_COLOR,
                        },
                    )?;
                    canvas.draw(&rect, DrawParam::default());
                }
            }
        }

        canvas.finish(ctx)?;
        Ok(())
    }

    fn mouse_button_down_event(
        &mut self,
        _ctx: &mut Context,
        button: MouseButton,
        x: f32,
        y: f32,
    ) -> GameResult {
        if button == MouseButton::Left {
            // Add a new ball at the mouse position with random velocity
            let mut ball = Body::new_circle(
                Point2::new(x as f64, y as f64),
                BALL_RADIUS as f64,
                Material::rubber(),
                BodyType::Dynamic,
            );
            // Add some initial velocity
            ball.velocity = nalgebra::Vector2::new(-5.0, -2.0);
            self.world.add_body(ball);
        }
        Ok(())
    }
}

fn main() -> GameResult {
    let cb = ggez::ContextBuilder::new("physics", "rust")
        .window_setup(ggez::conf::WindowSetup::default().title("Physics Engine"))
        .window_mode(ggez::conf::WindowMode::default().dimensions(WINDOW_WIDTH, WINDOW_HEIGHT));
    
    let (ctx, event_loop) = cb.build()?;
    let state = MainState::new()?;
    event::run(ctx, event_loop, state)
}
