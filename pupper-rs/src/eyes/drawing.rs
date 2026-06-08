use eframe::egui;
use egui::{Color32, Pos2, Shape, Stroke, Vec2};

pub fn quadratic_bezier_points(start: Pos2, ctrl: Pos2, end: Pos2, steps: usize) -> Vec<Pos2> {
    (0..=steps)
        .map(|i| {
            let t = i as f32 / steps as f32;
            let u = 1.0 - t;
            Pos2 {
                x: u * u * start.x + 2.0 * u * t * ctrl.x + t * t * end.x,
                y: u * u * start.y + 2.0 * u * t * ctrl.y + t * t * end.y,
            }
        })
        .collect()
}

pub fn draw_eye(painter: &egui::Painter, center: Pos2, pupil_offset: Vec2) {
    // Palette (tuned to the reference)
    let ring_outer = Color32::from_rgb(0x2a, 0x2f, 0x36); // dark gray ring
    let ring_blue = Color32::from_rgb(075, 149, 181); // bright blue ring
    let under_highlight_blue = Color32::from_rgb(50, 102, 136); // cyan-blue
    let gloss = Color32::from_rgba_premultiplied(255, 255, 255, (0.95 * 255.0) as u8);

    // Outer dark ring
    painter.add(Shape::circle_stroke(
        center,
        140.0,
        Stroke::new(8.0, ring_outer),
    ));

    // Thick blue bezel
    painter.add(Shape::circle_filled(center, 132.0, ring_blue));

    // Pupil / eye interior (with offset for tracking)
    let pupil_center = center + pupil_offset;
    painter.add(Shape::circle_filled(pupil_center, 100.0, Color32::BLACK));

    // under highlight (arc) - moves with pupil
    let radius = 80.0;
    let start_angle = 45.0_f32.to_radians();
    let end_angle = 135.0_f32.to_radians();
    let steps = 28;
    let arc_pts = (0..=steps)
        .map(|i| {
            let t = i as f32 / steps as f32;
            let angle = start_angle + t * (end_angle - start_angle);
            pupil_center + Vec2::new(radius * angle.cos(), radius * angle.sin())
        })
        .collect::<Vec<_>>();
    painter.add(Shape::line(
        arc_pts,
        Stroke::new(14.0, under_highlight_blue),
    ));
    // Little blue dot at the right end - moves with pupil
    painter.add(Shape::circle_filled(
        pupil_center + Vec2::new(76.0, 34.0),
        8.0,
        under_highlight_blue,
    ));

    // Gloss highlights (top-left) - moves with pupil
    painter.add(Shape::circle_filled(
        pupil_center + Vec2::new(-42.0, -54.0),
        26.0,
        gloss,
    ));
    painter.add(Shape::circle_filled(
        pupil_center + Vec2::new(-70.0, -8.0),
        12.0,
        gloss,
    ));
}

/// Draw a black mask over the lower portion of the eye to create a "smiling squint" look.
/// `squint` controls how much of the eye is covered: 0.0 = no mask, 1.0 = fully closed.
/// A good default for a gentle smile is around 0.55.
pub fn draw_smile_mask(painter: &egui::Painter, center: Pos2, squint: f32) {
    let squint = squint.clamp(0.0, 1.0);
    if squint < 0.01 {
        return;
    }

    let mask_color = Color32::BLACK;
    let eye_radius = 145.0;

    // The arc top edge: higher squint → arc moves further up into the eye.
    // At squint=0.55 the arc sits roughly at center-y, giving a natural smile.
    let arc_y = center.y + eye_radius * (1.0 - squint * 2.0);

    let arc_start = Pos2::new(center.x - eye_radius, center.y + 10.0);
    let arc_ctrl = Pos2::new(center.x, arc_y);
    let arc_end = Pos2::new(center.x + eye_radius, center.y + 10.0);

    let mut pts = quadratic_bezier_points(arc_start, arc_ctrl, arc_end, 32);
    // Close the polygon by adding bottom corners
    let bottom = center.y + eye_radius + 20.0;
    pts.push(Pos2::new(center.x + eye_radius, bottom));
    pts.push(Pos2::new(center.x - eye_radius, bottom));

    painter.add(Shape::convex_polygon(pts, mask_color, Stroke::NONE));
}

pub fn draw_eyebrow(painter: &egui::Painter, center: Pos2) {
    let brow_col = Color32::from_rgb(0x33, 0x36, 0x3c);
    let brow_start = center + Vec2::new(-88.0, -150.0);
    let brow_ctrl = center + Vec2::new(0.0, -195.0);
    let brow_end = center + Vec2::new(88.0, -150.0);
    let brow_pts = quadratic_bezier_points(brow_start, brow_ctrl, brow_end, 24);
    painter.add(Shape::line(brow_pts, Stroke::new(14.0, brow_col)));
}
