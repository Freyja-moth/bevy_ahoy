use avian3d::{
    character_controller::move_and_slide::MoveHitData,
    parry::shape::{Capsule, SharedShape},
};
use bevy_ecs::{
    entity::EntityHashSet, intern::Interned, lifecycle::HookContext,
    relationship::RelationshipSourceCollection as _, schedule::ScheduleLabel, world::DeferredWorld,
};
use core::time::Duration;
use std::sync::Arc;
use tracing::error;

use crate::{input::AccumulatedInput, prelude::*};

pub(super) fn plugin(schedule: Interned<dyn ScheduleLabel>) -> impl Fn(&mut App) {
    move |app: &mut App| {
        app.add_systems(schedule, run_kcc.in_set(AhoySystems::MoveCharacters));
    }
}

#[derive(Component, Clone, Reflect, Debug)]
#[reflect(Component)]
#[require(
    AccumulatedInput,
    CharacterControllerState,
    TranslationInterpolation,
    RigidBody = RigidBody::Kinematic,
    Collider = Collider::cylinder(0.7, 1.8)
)]
#[component(on_add=CharacterController::on_add)]
pub struct CharacterController {
    pub crouch_height: f32,
    pub filter: SpatialQueryFilter,
    pub standing_view_height: f32,
    pub crouch_view_height: f32,
    pub ground_distance: f32,
    pub jump_detection_speed: f32,
    pub min_walk_cos: f32,
    pub stop_speed: f32,
    pub friction_hz: f32,
    pub acceleration_hz: f32,
    pub gravity: f32,
    pub step_size: f32,
    pub jump_speed: f32,
    pub crouch_scale: f32,
    pub speed: f32,
    pub air_speed: f32,
    pub move_and_slide: MoveAndSlideConfig,
}

impl Default for CharacterController {
    fn default() -> Self {
        Self {
            crouch_height: 1.3,
            filter: SpatialQueryFilter::default(),
            standing_view_height: 1.7,
            crouch_view_height: 1.2,
            ground_distance: 0.02,
            jump_detection_speed: 0.5,
            min_walk_cos: 0.766,
            stop_speed: 5.0,
            friction_hz: 10.0,
            acceleration_hz: 12.0,
            gravity: 36.0,
            step_size: 1.0,
            jump_speed: 12.0,
            crouch_scale: 0.3,
            speed: 15.5,
            air_speed: 1.5,
            move_and_slide: MoveAndSlideConfig {
                skin_width: 0.01,
                ..default()
            },
        }
    }
}

impl CharacterController {
    pub fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        {
            let Some(mut kcc) = world.get_mut::<Self>(ctx.entity) else {
                return;
            };
            kcc.filter.excluded_entities.add(ctx.entity);
        }

        let crouch_height = {
            let Some(kcc) = world.get::<Self>(ctx.entity) else {
                return;
            };
            kcc.crouch_height
        };

        let Some(collider) = world.entity(ctx.entity).get::<Collider>().cloned() else {
            return;
        };
        let standing_aabb = collider.aabb(default(), Rotation::default());
        let standing_height = standing_aabb.max.y - standing_aabb.min.y;

        let Some(mut state) = world.get_mut::<CharacterControllerState>(ctx.entity) else {
            return;
        };
        state.standing_collider = collider.clone();

        let frac = crouch_height / standing_height;

        let mut crouching_collider = Collider::from(SharedShape(Arc::from(
            state.standing_collider.shape().clone_dyn(),
        )));

        if crouching_collider.shape().as_capsule().is_some() {
            let capsule = crouching_collider
                .shape_mut()
                .make_mut()
                .as_capsule_mut()
                .unwrap();
            let radius = capsule.radius;
            let new_height = (crouch_height - radius).max(0.0);
            *capsule = Capsule::new_y(new_height / 2.0, radius);
        } else {
            // note: well-behaved shapes like cylinders and cuboids will not actually subdivide when scaled, yay
            crouching_collider.set_scale(vec3(1.0, frac, 1.0), 16);
        }
        state.crouching_collider = Collider::compound(vec![(
            Vec3::Y * (crouch_height - standing_height) / 2.0,
            Rotation::default(),
            crouching_collider,
        )])
    }
}

#[derive(Component, Clone, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct CharacterControllerState {
    pub velocity: Vec3,
    #[reflect(ignore)]
    pub standing_collider: Collider,
    #[reflect(ignore)]
    pub crouching_collider: Collider,
    pub previous_grounded: Option<MoveHitData>,
    pub grounded: Option<MoveHitData>,
    pub crouching: bool,
    pub ground_plane: bool,
    pub grounded_entity: Option<Entity>,
    pub walking: bool,
    pub touching_entities: EntityHashSet,
}

impl CharacterControllerState {
    pub fn collider(&self) -> &Collider {
        if self.crouching {
            &self.crouching_collider
        } else {
            &self.standing_collider
        }
    }
}

fn run_kcc(
    mut kccs: Query<(
        Entity,
        &CharacterController,
        &mut CharacterControllerState,
        &AccumulatedInput,
        &mut Transform,
        &Collider,
        Option<&CharacterControllerCamera>,
    )>,
    cams: Query<&Transform, Without<CharacterController>>,
    time: Res<Time>,
    move_and_slide: MoveAndSlide,
    mut commands: Commands,
) {
    for (entity, cfg, mut state, input, mut transform, mut collider, cam) in &mut kccs {
        state.touching_entities.clear();
        let original_transform = *transform;

        let ctx = Ctx {
            orientation: cam
                .and_then(|e| cams.get(e.get()).copied().ok())
                .unwrap_or(*transform),
            cfg: cfg.clone(),
            input: *input,
            dt: time.delta(),
        };

        // here we'd handle things like spectator, dead, noclip, etc.

        check_duck(*transform, &move_and_slide, &mut state, &ctx);

        ground_trace(*transform, &move_and_slide, &mut state, &ctx);

        let wish_velocity = calculate_wish_velocity(&state, &ctx);
        if state.walking {
            walk_move(
                &mut transform,
                wish_velocity,
                &move_and_slide,
                &mut state,
                &ctx,
            )
        } else {
            air_move(
                &mut transform,
                wish_velocity,
                &move_and_slide,
                &mut state,
                &ctx,
            )
        };
        ground_trace(*transform, &move_and_slide, &mut state, &ctx);

        dejitter_output(&mut transform, original_transform);

        if !Arc::ptr_eq(&collider.shape().0, &state.collider().shape().0) {
            commands.entity(entity).insert(state.collider().clone());
        }
    }
}

#[derive(Debug)]
struct Ctx {
    orientation: Transform,
    cfg: CharacterController,
    input: AccumulatedInput,
    dt: Duration,
}

fn calculate_wish_velocity(state: &CharacterControllerState, ctx: &Ctx) -> Vec3 {
    let movement = ctx.input.last_movement.unwrap_or_default();
    let mut forward = Vec3::from(ctx.orientation.forward());
    forward.y = 0.0;
    forward = forward.normalize_or_zero();
    if let Some(grounded) = state.grounded {
        forward = MoveAndSlide::project_velocity(forward, &[grounded.normal1.try_into().unwrap()]);
    }
    forward = forward.normalize_or_zero();
    let mut right = Vec3::from(ctx.orientation.right());
    right.y = 0.0;
    if let Some(grounded) = state.grounded {
        right = MoveAndSlide::project_velocity(right, &[grounded.normal1.try_into().unwrap()]);
    }
    right = right.normalize_or_zero();

    let wish_vel = movement.y * forward + movement.x * right;
    let wish_dir = wish_vel.normalize_or_zero();

    // clamp the speed lower if ducking
    let speed = if state.crouching {
        ctx.cfg.speed * ctx.cfg.crouch_scale
    } else {
        ctx.cfg.speed
    };
    wish_dir * speed
}

fn dejitter_output(transform: &mut Transform, original_transform: Transform) {
    const EPSILON: f32 = 0.0005;

    for i in 0..3 {
        let delta_pos = original_transform.translation - transform.translation;
        if delta_pos[i].abs() < EPSILON {
            transform.translation[i] = original_transform.translation[i];
        }
    }
}

fn walk_move(
    transform: &mut Transform,
    wish_velocity: Vec3,
    move_and_slide: &MoveAndSlide,
    state: &mut CharacterControllerState,
    ctx: &Ctx,
) {
    let jumped: bool;
    jumped = check_jump(state, ctx);
    if jumped {
        air_move(transform, wish_velocity, move_and_slide, state, ctx);
        return;
    }

    friction(state, ctx);

    accelerate(wish_velocity, state, ctx);

    if let Some(grounded) = state.grounded {
        state.velocity =
            MoveAndSlide::project_velocity(state.velocity, &[grounded.normal1.try_into().unwrap()]);
    }

    // don't do anything if standing still
    if state.velocity.xz() == Vec2::ZERO {
        return;
    }

    step_slide_move(transform, move_and_slide, state, ctx);
}

#[must_use]
fn check_jump(state: &mut CharacterControllerState, ctx: &Ctx) -> bool {
    if !ctx.input.jumped {
        return false;
    }
    state.ground_plane = false;
    state.walking = false;
    state.velocity.y = ctx.cfg.jump_speed;
    // trigger jump event
    true
}

fn air_move(
    transform: &mut Transform,
    wish_velocity: Vec3,
    move_and_slide: &MoveAndSlide,
    state: &mut CharacterControllerState,
    ctx: &Ctx,
) {
    friction(state, ctx);

    // not on ground, so little effect on velocity
    air_accelerate(wish_velocity, state, ctx);
    state.velocity += Vec3::NEG_Y * ctx.cfg.gravity * ctx.dt.as_secs_f32();

    // we may have a ground plane that is very steep, even
    // though we don't have a groundentity
    // slide along the steep plane
    if state.ground_plane
        && let Some(grounded) = state.grounded
    {
        state.velocity =
            MoveAndSlide::project_velocity(state.velocity, &[grounded.normal1.try_into().unwrap()]);
    }
    step_slide_move(transform, move_and_slide, state, ctx);
}

fn step_slide_move(
    transform: &mut Transform,
    move_and_slide: &MoveAndSlide,
    state: &mut CharacterControllerState,
    ctx: &Ctx,
) {
    let start_o = *transform;
    let start_v = state.velocity;

    let mut clipped = false;
    let move_and_slide_cfg = if let Some(grounded) = state.grounded {
        let mut cfg = ctx.cfg.move_and_slide.clone();
        cfg.planes.push(Dir3::new_unchecked(grounded.normal1));
        cfg
    } else {
        ctx.cfg.move_and_slide.clone()
    };
    let mut direct_collisions = EntityHashSet::new();
    let result = move_and_slide.move_and_slide(
        state.collider(),
        transform.translation,
        transform.rotation,
        state.velocity,
        ctx.dt,
        &move_and_slide_cfg,
        &ctx.cfg.filter,
        |hit| {
            clipped = true;
            direct_collisions.insert(hit.entity);
            true
        },
    );
    state.touching_entities = direct_collisions;

    transform.translation = result.position;
    state.velocity = result.projected_velocity;

    // Non-Quake: also don't step in the air
    if !clipped || !state.walking {
        // we got exactly where we wanted to go first try);
        return;
    }
    let direct_transform = *transform;
    let direct_velocity = state.velocity;

    let cast_dir = Dir3::NEG_Y;
    let cast_dist = ctx.cfg.step_size;
    let trace = move_and_slide.cast_move(
        state.collider(),
        start_o.translation,
        start_o.rotation,
        cast_dir * cast_dist,
        ctx.cfg.move_and_slide.skin_width,
        &ctx.cfg.filter,
    );

    // never step up when you still have up velocity
    if state.velocity.y > 0.0
        && (trace.is_none() || trace.is_some_and(|t| t.normal1.dot(Vec3::Y) < ctx.cfg.min_walk_cos))
    {
        return;
    }

    let cast_dir = Dir3::Y;
    // test the player position if they were a stepheight higher
    let trace = move_and_slide.cast_move(
        state.collider(),
        start_o.translation,
        start_o.rotation,
        cast_dir * cast_dist,
        ctx.cfg.move_and_slide.skin_width,
        &ctx.cfg.filter,
    );
    let step_size = if let Some(trace) = trace {
        trace.distance
    } else {
        cast_dist
    };
    if step_size <= 0.0 {
        // can't step up
        return;
    }

    // try slidemove from this position
    transform.translation = start_o.translation + cast_dir * step_size;
    transform.translation += move_and_slide.depenetrate_all(
        state.collider(),
        transform.translation,
        transform.rotation,
        &((&ctx.cfg.move_and_slide).into()),
        &ctx.cfg.filter,
    );

    state.velocity = start_v;
    let mut step_collisions = EntityHashSet::new();
    let result = move_and_slide.move_and_slide(
        state.collider(),
        transform.translation,
        transform.rotation,
        state.velocity,
        ctx.dt,
        &move_and_slide_cfg,
        &ctx.cfg.filter,
        |hit| {
            step_collisions.insert(hit.entity);
            true
        },
    );
    transform.translation = result.position;
    state.velocity = result.projected_velocity;

    // push down the final amount
    let cast_dir = Dir3::NEG_Y;
    let cast_dist = step_size;
    let trace = move_and_slide.cast_move(
        state.collider(),
        transform.translation,
        transform.rotation,
        cast_dir * cast_dist,
        ctx.cfg.move_and_slide.skin_width,
        &ctx.cfg.filter,
    );
    if let Some(trace) = trace {
        transform.translation += cast_dir * trace.distance;
        state.velocity =
            MoveAndSlide::project_velocity(state.velocity, &[trace.normal1.try_into().unwrap()]);
    } else {
        transform.translation += cast_dir * cast_dist;
    }

    transform.translation += move_and_slide.depenetrate_all(
        state.collider(),
        transform.translation,
        transform.rotation,
        &((&ctx.cfg.move_and_slide).into()),
        &ctx.cfg.filter,
    );

    // non-Quake code incoming: if we
    // - didn't really step up
    // - stepped onto something we would slide down from
    // let's not step at all. That eliminates nasty situations where we get "ghost steps" when penetrating walls.
    let direct_horizontal_dist = start_o
        .translation
        .xz()
        .distance_squared(direct_transform.translation.xz());
    let step_horizontal_dist = start_o
        .translation
        .xz()
        .distance_squared(transform.translation.xz());
    let did_not_advance_through_stepping = direct_horizontal_dist >= step_horizontal_dist - 0.001;

    if did_not_advance_through_stepping || trace.is_some_and(|t| t.normal1.y < ctx.cfg.min_walk_cos)
    {
        *transform = direct_transform;
        state.velocity = direct_velocity;
    } else {
        state.touching_entities = step_collisions;
    }
}

fn accelerate(wish_velocity: Vec3, state: &mut CharacterControllerState, ctx: &Ctx) {
    let (wish_dir, wish_speed) = Dir3::new_and_length(wish_velocity).unwrap_or((Dir3::NEG_Z, 0.0));
    let current_speed = state.velocity.dot(wish_dir.into());
    let add_speed = wish_speed - current_speed;
    if add_speed <= 0.0 {
        return;
    }

    let accel_speed = f32::min(
        ctx.cfg.acceleration_hz * ctx.dt.as_secs_f32() * wish_speed,
        add_speed,
    );
    state.velocity += accel_speed * wish_dir;
}

fn air_accelerate(wish_velocity: Vec3, state: &mut CharacterControllerState, ctx: &Ctx) {
    let (wish_dir, wish_speed) = Dir3::new_and_length(wish_velocity).unwrap_or((Dir3::NEG_Z, 0.0));
    let current_speed = state.velocity.dot(wish_dir.into());
    // right here is where air strafing happens: `current_speed` is close to 0 when we want to move perpendicular to
    // our current velocity, making `add_speed` large.
    let air_wish_speed = f32::min(wish_speed, ctx.cfg.air_speed);
    let add_speed = air_wish_speed - current_speed;
    if add_speed <= 0.0 {
        return;
    }

    let accel_speed = f32::min(
        ctx.cfg.acceleration_hz * ctx.dt.as_secs_f32() * wish_speed,
        add_speed,
    );
    state.velocity += accel_speed * wish_dir;
}

fn friction(state: &mut CharacterControllerState, ctx: &Ctx) {
    let mut vec = state.velocity;
    if state.walking {
        // ignore slope movement
        vec.y = 0.0;
    }
    let speed = vec.length();
    if speed < 0.05 {
        state.velocity.x = 0.0;
        state.velocity.z = 0.0;
        return;
    }
    let drop = if state.walking {
        let stop_speed = f32::max(speed, ctx.cfg.stop_speed);
        stop_speed * ctx.cfg.friction_hz * ctx.dt.as_secs_f32()
    } else {
        0.0
    };

    let new_speed = f32::max(speed - drop, 0.0);
    state.velocity = state.velocity / speed * new_speed;
}

fn check_duck(
    transform: Transform,
    move_and_slide: &MoveAndSlide,
    state: &mut CharacterControllerState,
    ctx: &Ctx,
) {
    if ctx.input.crouched {
        state.crouching = true;
    } else if state.crouching {
        // try to stand up
        state.crouching = false;
        let is_intersecting = is_intersecting(transform, state, move_and_slide, ctx);
        state.crouching = is_intersecting;
    }
}

fn ground_trace(
    transform: Transform,
    move_and_slide: &MoveAndSlide,
    state: &mut CharacterControllerState,
    ctx: &Ctx,
) {
    let cast_dir = Dir3::NEG_Y;
    let cast_dist = ctx.cfg.ground_distance;
    let trace = move_and_slide.cast_move(
        state.collider(),
        transform.translation,
        transform.rotation,
        cast_dir * cast_dist,
        ctx.cfg.move_and_slide.skin_width,
        &ctx.cfg.filter,
    );
    state.previous_grounded = state.grounded;
    state.grounded = trace;

    // if the trace didn't hit anything, we are in free fall
    let Some(trace) = trace else {
        ground_trace_missed();
        state.grounded_entity = None;
        state.ground_plane = false;
        state.walking = false;
        return;
    };

    // check if getting thrown off the ground
    if state.velocity.y > 0.0 && state.velocity.dot(trace.normal1) > ctx.cfg.jump_detection_speed {
        // here we could trigger a jump start event
        state.grounded_entity = None;
        state.ground_plane = false;
        state.walking = false;
        return;
    }

    // slopes that are too steep will not be considered onground
    if trace.normal1.y < ctx.cfg.min_walk_cos {
        state.grounded_entity = None;
        state.ground_plane = true;
        state.walking = false;
        return;
    }

    state.ground_plane = true;
    state.walking = true;
    if state.grounded_entity.is_none() {
        // trigger landing event
        crash_land()
    }
    state.grounded_entity = Some(trace.entity);
}

fn ground_trace_missed() {
    // here we can
    // - trigger transitions into free-fall
    // - do a trace if we are falling quite a bit
    // - if so, trigger the appropriate falling animation
}

fn crash_land() {
    // here we can
    // - check how hard we crashed
    // - trigger crash landing event
    //   - deal damage
    //   - play anims
    //   - reset bob cycle
}

#[must_use]
fn is_intersecting(
    transform: Transform,
    state: &CharacterControllerState,
    move_and_slide: &MoveAndSlide,
    ctx: &Ctx,
) -> bool {
    let mut intersecting = false;
    // No need to worry about skin width, depenetration will take care of it.
    // If we used skin width, we could not stand up if we are closer than skin width to the ground,
    // which happens when going under a slope.
    move_and_slide.query_pipeline.shape_intersections_callback(
        state.collider(),
        transform.translation,
        transform.rotation,
        &ctx.cfg.filter,
        |_| {
            intersecting = true;
            false
        },
    );
    intersecting
}
