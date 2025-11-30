use avian_pickup::input::{AvianPickupAction, AvianPickupInput};
use bevy_time::Stopwatch;

use crate::prelude::*;

use crate::fixed_update_utils::did_fixed_timestep_run_this_frame;

pub(super) fn plugin(app: &mut App) {
    app.add_observer(apply_movement)
        .add_observer(apply_jump)
        .add_observer(apply_crouch)
        .add_observer(apply_drop)
        .add_observer(apply_pull)
        .add_observer(apply_throw)
        .add_systems(
            RunFixedMainLoop,
            clear_accumulated_input
                .run_if(did_fixed_timestep_run_this_frame)
                .in_set(RunFixedMainLoopSystems::AfterFixedMainLoop),
        )
        .add_systems(PreUpdate, tick_timers.in_set(EnhancedInputSystems::Update));
}

#[derive(Debug, InputAction)]
#[action_output(Vec2)]
pub struct Movement;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct Jump;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct Crouch;

#[derive(Debug, InputAction)]
#[action_output(Vec2)]
pub struct RotateCamera;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct PullObject;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct DropObject;

#[derive(Debug, InputAction)]
#[action_output(bool)]
pub struct ThrowObject;

/// Input accumulated since the last fixed update loop. Is cleared after every fixed update loop.
#[derive(Component, Clone, Reflect, Default, Debug)]
#[reflect(Component)]
pub struct AccumulatedInput {
    // The last non-zero move that was input since the last fixed update loop
    pub last_movement: Option<Vec2>,
    // Time since the last jump input. Will be `None` once the jump was processed.
    pub jumped: Option<Stopwatch>,
    // Whether any frame since the last fixed update loop input a crouch
    pub crouched: bool,
}

fn apply_movement(
    movement: On<Fire<Movement>>,
    mut accumulated_inputs: Query<&mut AccumulatedInput>,
) {
    if let Ok(mut accumulated_inputs) = accumulated_inputs.get_mut(movement.context) {
        accumulated_inputs.last_movement = Some(movement.value);
    }
}

fn apply_jump(jump: On<Fire<Jump>>, mut accumulated_inputs: Query<&mut AccumulatedInput>) {
    if let Ok(mut accumulated_inputs) = accumulated_inputs.get_mut(jump.context) {
        accumulated_inputs.jumped = Some(Stopwatch::new());
    }
}

fn apply_crouch(crouch: On<Fire<Crouch>>, mut accumulated_inputs: Query<&mut AccumulatedInput>) {
    if let Ok(mut accumulated_inputs) = accumulated_inputs.get_mut(crouch.context) {
        accumulated_inputs.crouched = true;
    }
}

fn apply_pull(
    crouch: On<Fire<PullObject>>,
    mut avian_pickup_input_writer: MessageWriter<AvianPickupInput>,
    cams: Query<&CharacterControllerCamera>,
) {
    let actor = if let Ok(camera) = cams.get(crouch.context) {
        camera.get()
    } else {
        crouch.context
    };
    avian_pickup_input_writer.write(AvianPickupInput {
        action: AvianPickupAction::Pull,
        actor,
    });
}

fn apply_drop(
    crouch: On<Fire<DropObject>>,
    mut avian_pickup_input_writer: MessageWriter<AvianPickupInput>,
    cams: Query<&CharacterControllerCamera>,
) {
    let actor = if let Ok(camera) = cams.get(crouch.context) {
        camera.get()
    } else {
        crouch.context
    };
    avian_pickup_input_writer.write(AvianPickupInput {
        action: AvianPickupAction::Drop,
        actor,
    });
}

fn apply_throw(
    crouch: On<Fire<ThrowObject>>,
    mut avian_pickup_input_writer: MessageWriter<AvianPickupInput>,
    cams: Query<&CharacterControllerCamera>,
) {
    let actor = if let Ok(camera) = cams.get(crouch.context) {
        camera.get()
    } else {
        crouch.context
    };
    avian_pickup_input_writer.write(AvianPickupInput {
        action: AvianPickupAction::Throw,
        actor,
    });
}

fn clear_accumulated_input(mut accumulated_inputs: Query<&mut AccumulatedInput>) {
    for mut accumulated_input in &mut accumulated_inputs {
        *accumulated_input = AccumulatedInput {
            last_movement: default(),
            jumped: accumulated_input.jumped.clone(),
            crouched: default(),
        }
    }
}

fn tick_timers(mut inputs: Query<&mut AccumulatedInput>, time: Res<Time>) {
    for mut input in inputs.iter_mut() {
        if let Some(jumped) = input.jumped.as_mut() {
            jumped.tick(time.delta());
        }
    }
}
