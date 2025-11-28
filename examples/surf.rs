use avian3d::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, world::DeferredWorld},
    gltf::GltfPlugin,
    image::{ImageAddressMode, ImageSamplerDescriptor},
    input::common_conditions::input_just_pressed,
    light::{CascadeShadowConfigBuilder, DirectionalLightShadowMap},
    log::{LogPlugin, tracing_subscriber::field::MakeExt},
    pbr::Atmosphere,
    prelude::*,
    scene::SceneInstanceReady,
    window::{CursorGrabMode, CursorOptions, WindowResolution},
};
use bevy_ahoy::{kcc::CharacterControllerState, prelude::*};
use bevy_enhanced_input::prelude::{Release, *};
use bevy_mod_mipmap_generator::{MipmapGeneratorPlugin, generate_mipmaps};
use bevy_trenchbroom::{physics::SceneCollidersReady, prelude::*};
use bevy_trenchbroom_avian::AvianPhysicsBackend;
use core::ops::Deref;

use crate::util::ExampleUtilPlugin;

mod util;

fn main() -> AppExit {
    App::new()
        .add_plugins((
            DefaultPlugins
                .set(GltfPlugin {
                    use_model_forward_direction: true,
                    ..default()
                })
                .set(LogPlugin {
                    filter: format!(
                        concat!(
                            "{default},",
                            "symphonia_bundle_mp3::demuxer=warn,",
                            "symphonia_format_caf::demuxer=warn,",
                            "symphonia_format_isompf4::demuxer=warn,",
                            "symphonia_format_mkv::demuxer=warn,",
                            "symphonia_format_ogg::demuxer=warn,",
                            "symphonia_format_riff::demuxer=warn,",
                            "symphonia_format_wav::demuxer=warn,",
                            "bevy_trenchbroom::physics=off,",
                            "calloop::loop_logic=error,",
                        ),
                        default = bevy::log::DEFAULT_FILTER
                    ),
                    fmt_layer: |_| {
                        Some(Box::new(
                            bevy::log::tracing_subscriber::fmt::Layer::default()
                                .map_fmt_fields(MakeExt::debug_alt)
                                .with_writer(std::io::stderr),
                        ))
                    },
                    ..default()
                })
                .set(ImagePlugin {
                    default_sampler: ImageSamplerDescriptor {
                        address_mode_u: ImageAddressMode::Repeat,
                        address_mode_v: ImageAddressMode::Repeat,
                        address_mode_w: ImageAddressMode::Repeat,
                        ..ImageSamplerDescriptor::linear()
                    },
                })
                .set(WindowPlugin {
                    primary_window: Window {
                        resolution: WindowResolution::new(1920, 1080),
                        ..default()
                    }
                    .into(),
                    ..default()
                }),
            PhysicsPlugins::default(),
            EnhancedInputPlugin,
            AhoyPlugin::default(),
            TrenchBroomPlugins(
                TrenchBroomConfig::new("bevy_ahoy_surf")
                    .default_solid_scene_hooks(|| {
                        SceneHooks::new()
                            .convex_collider()
                            .smooth_by_default_angle()
                    })
                    .auto_remove_textures(
                        [
                            "clip",
                            "skip",
                            "__TB_empty",
                            "utopia/nodraw",
                            "tools/tool_trigger",
                        ]
                        .into_iter()
                        .map(String::from)
                        .collect::<std::collections::HashSet<_>>(),
                    ),
            ),
            TrenchBroomPhysicsPlugin::new(AvianPhysicsBackend),
            ExampleUtilPlugin,
        ))
        .add_input_context::<PlayerInput>()
        .add_systems(Startup, setup)
        .add_observer(spawn_player)
        .add_systems(
            Update,
            (
                capture_cursor.run_if(input_just_pressed(MouseButton::Left)),
                release_cursor.run_if(input_just_pressed(KeyCode::Escape)),
            ),
        )
        .run()
}

fn setup(mut commands: Commands, assets: Res<AssetServer>) {
    commands.spawn(SceneRoot(assets.load("maps/utopia.map#Scene")));
    commands.spawn(Camera3d::default());
}

#[derive(Component, Reflect, Debug)]
#[reflect(Component)]
struct Player;

#[point_class(base(Transform, Visibility))]
#[reflect(Component)]
struct SpawnPlayer;

fn spawn_player(
    insert: On<Insert, SpawnPlayer>,
    players: Query<Entity, With<Player>>,
    spawner: Query<&Transform>,
    camera: Single<Entity, With<Camera3d>>,
    mut commands: Commands,
) {
    for player in players {
        // Respawn the player on hot-reloads
        commands.entity(player).despawn();
    }
    let Ok(transform) = spawner.get(insert.entity).copied() else {
        return;
    };
    let player = commands
        .spawn((
            Player,
            transform,
            PlayerInput,
            CharacterController {
                acceleration_hz: 10.0,
                air_acceleration_hz: 150.0,
                ..default()
            },
            RigidBody::Kinematic,
            Collider::cylinder(0.7, 1.8),
        ))
        .id();
    commands
        .entity(camera.into_inner())
        .insert(CharacterControllerCameraOf(player));
}

#[derive(Component, Default)]
#[component(on_add = PlayerInput::on_add)]
pub(crate) struct PlayerInput;

impl PlayerInput {
    fn on_add(mut world: DeferredWorld, ctx: HookContext) {
        world
            .commands()
            .entity(ctx.entity)
            .insert(actions!(PlayerInput[
                (
                    Action::<Movement>::new(),
                    DeadZone::default(),
                    Bindings::spawn((
                        Cardinal::wasd_keys(),
                        Axial::left_stick()
                    ))
                ),
                (
                    Action::<Jump>::new(),
                    bindings![KeyCode::Space,  GamepadButton::South],
                ),
                (
                    Action::<Crouch>::new(),
                    bindings![KeyCode::ControlLeft, GamepadButton::LeftTrigger],
                ),
                (
                    Action::<RotateCamera>::new(),
                    Scale::splat(0.04),
                    Bindings::spawn((
                        Spawn(Binding::mouse_motion()),
                        Axial::right_stick()
                    ))
                ),
            ]));
    }
}

#[solid_class(base(Transform, Visibility), hooks(SceneHooks::new().smooth_by_default_angle()))]
#[component(on_add = Self::on_add_prop)]
struct FuncIllusionary;

impl FuncIllusionary {
    fn on_add_prop(mut world: DeferredWorld, ctx: HookContext) {
        if world.is_scene_world() {
            return;
        }
    }
}

#[solid_class(base(Transform, Visibility), hooks(SceneHooks::new().smooth_by_default_angle()))]
#[component(on_add = Self::on_add_prop)]
struct TriggerTeleport;

impl TriggerTeleport {
    fn on_add_prop(mut world: DeferredWorld, ctx: HookContext) {
        if world.is_scene_world() {
            return;
        }
    }
}

#[solid_class(base(Transform, Visibility), hooks(SceneHooks::new().smooth_by_default_angle()))]
#[component(on_add = Self::on_add_prop)]
#[derive(Default)]
struct TriggerPush {
    speed: f32,
}

impl TriggerPush {
    fn on_add_prop(mut world: DeferredWorld, ctx: HookContext) {
        if world.is_scene_world() {
            return;
        }
    }
}

fn capture_cursor(mut cursor: Single<&mut CursorOptions>) {
    cursor.grab_mode = CursorGrabMode::Locked;
    cursor.visible = false;
}

fn release_cursor(mut cursor: Single<&mut CursorOptions>) {
    cursor.visible = true;
    cursor.grab_mode = CursorGrabMode::None;
}
