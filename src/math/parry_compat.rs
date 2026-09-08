//! Conversion helpers between Avian's `bevy_math`-backed math types and the
//! math types re-exported by `parry` (`pub extern crate parry3d as parry;` /
//! `parry2d as parry;` in `lib.rs`).
//!
//! `parry` resolves its own `glam` dependency, which is not guaranteed to be
//! (and, as of this writing, is NOT) the same semver-major version of `glam`
//! that `bevy_math` resolves. Cargo therefore treats `bevy_math::Vec3`/`Vec2`
//! and `parry::math::Vector` as two distinct, incompatible types, even though
//! they are structurally identical (the same `x`/`y`(`/z`) `f32` fields).
//!
//! Real, live compile error this shim fixes (`spatial_query::system_param`):
//!
//! ```text
//! error[E0308]: mismatched types
//!    --> normal2: pose2.rotation * hit.normal2,
//!     = note: expected struct `bevy_math::Vec3`
//!                found struct `parry::math::Vec3`
//!     = note: two different versions of crate `glam` are being used;
//!             two types coming from two different versions of the same
//!             crate are different types even if they look the same
//! ```
//!
//! Rust's orphan rules block a blanket `impl From<parry::math::Vector> for
//! Vector` here -- neither type is local to this crate -- so these are plain
//! field-copy free functions instead. This is a real, deliberate fix per the
//! `magicborn-kingdom-avian3d-glam-version-conflict` task: stop chasing
//! version-pin alignment across the whole dependency graph (parry pulls in
//! its own `glam` transitively via its own Cargo.toml, independent of
//! whatever `[patch]`/pin avian3d's own consumers apply to `glam` for
//! `bevy_math`) and instead convert at the actual boundary call sites, since
//! the underlying field layout is identical regardless of crate version.
//!
//! Every call site where a `parry` query/shape result (a vector, a point, a
//! ray argument, etc.) crosses into Avian's own bevy-native [`Vector`] --  or
//! vice versa -- must go through one of these functions rather than relying
//! on the two types happening to unify.

#[allow(unused_imports)]
use super::{IVector, Quaternion, Vector};

/// Converts a `parry`-native (`glamx`/glam-0.30-backed) quaternion into Avian's own
/// (bevy_math-backed) [`Quaternion`].
///
/// Real, live compile error this fixes (multiple call sites, task
/// `magicborn-kingdom-avian3d-glam-version-conflict`): a `parry::math::Pose3`/`Isometry`
/// value's `.rotation` field (a `parry`-native `Quat`) flowing directly into bevy-facing
/// code (e.g. `make_pose(.., pose.rotation)`, `pose.rotation * some_bevy_vec3`) as if it
/// were a `bevy_math::Quat` -- the earlier per-call-site `Vec3` conversions alone did not
/// fix this because the surrounding POSE/QUAT STRUCT was still parry-native at the point
/// it was read back out.
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub(crate) fn parry_quat_to_bevy(q: parry::math::Rot3) -> Quaternion {
    Quaternion::from_xyzw(q.x, q.y, q.z, q.w)
}

/// Converts Avian's own (bevy_math-backed) [`Quaternion`] into a `parry`-native
/// (`glamx`/glam-0.30-backed) quaternion.
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub(crate) fn bevy_quat_to_parry(q: Quaternion) -> parry::math::Rot3 {
    parry::math::Rot3::from_xyzw(q.x, q.y, q.z, q.w)
}

/// Converts a whole `parry`-native pose (`Isometry`/`Pose3`: translation + rotation) into
/// Avian's own (bevy_math-backed) `(Vector, Quaternion)` pair, in one boundary conversion,
/// rather than converting `.translation`/`.rotation` field reads scattered downstream.
#[inline]
#[must_use]
#[cfg(feature = "3d")]
#[allow(dead_code)] // General-purpose boundary helper; not every call site needs the pair.
pub(crate) fn parry_pose_to_bevy(pose: parry::math::Pose3) -> (Vector, Quaternion) {
    (
        parry_vector_to_bevy(pose.translation),
        parry_quat_to_bevy(pose.rotation),
    )
}

/// Converts a `parry`-native vector/point into Avian's own (bevy_math-backed) [`Vector`].
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub(crate) fn parry_vector_to_bevy(v: parry::math::Vector) -> Vector {
    Vector::new(v.x, v.y, v.z)
}

/// Converts Avian's own (bevy_math-backed) [`Vector`] into a `parry`-native vector/point.
#[inline]
#[must_use]
#[cfg(feature = "3d")]
pub(crate) fn bevy_vector_to_parry(v: Vector) -> parry::math::Vector {
    parry::math::Vector::new(v.x, v.y, v.z)
}

/// Converts a `parry`-native vector/point into Avian's own (bevy_math-backed) [`Vector`].
#[inline]
#[must_use]
#[cfg(feature = "2d")]
pub(crate) fn parry_vector_to_bevy(v: parry::math::Vector) -> Vector {
    Vector::new(v.x, v.y)
}

/// Converts Avian's own (bevy_math-backed) [`Vector`] into a `parry`-native vector/point.
#[inline]
#[must_use]
#[cfg(feature = "2d")]
pub(crate) fn bevy_vector_to_parry(v: Vector) -> parry::math::Vector {
    parry::math::Vector::new(v.x, v.y)
}

/// Bulk-converts a slice of Avian's own (bevy_math-backed) [`Vector`]s into a freshly
/// allocated `Vec` of `parry`-native vectors/points.
///
/// Real, live compile errors this fixes (`collision::collider::parry::mod`): the mesh/
/// voxel/convex-hull constructors (`SharedShape::trimesh`, `::convex_decomposition[_with_params]`,
/// `::convex_hull`, `::convex_polyline`, `::polyline`, `::voxels_from_points`,
/// `::voxelized_mesh`, `::voxelized_convex_decomposition_with_params`, etc.) all take
/// `Vec<parry::math::Vector>` or `&[parry::math::Vector]` -- never Avian's own bevy-native
/// `Vec<Vector>`/`&[Vector]` -- so every one of those call sites needs the vertex/point
/// buffer converted at the boundary, same as any other `parry`-crossing value.
#[inline]
#[must_use]
pub(crate) fn bevy_vector_slice_to_parry(v: &[Vector]) -> Vec<parry::math::Vector> {
    v.iter().copied().map(bevy_vector_to_parry).collect()
}

/// Bulk-converts an owned `Vec` of Avian's own (bevy_math-backed) [`Vector`]s into a `Vec`
/// of `parry`-native vectors/points, consuming the input. See [`bevy_vector_slice_to_parry`]
/// for the borrowing counterpart and the real compile errors both fix.
#[inline]
#[must_use]
pub(crate) fn bevy_vector_vec_to_parry(v: Vec<Vector>) -> Vec<parry::math::Vector> {
    v.into_iter().map(bevy_vector_to_parry).collect()
}

/// Converts Avian's own (bevy_math-backed) `i32` grid-coordinate vector ([`IVector`]) into a
/// `parry`-native (`glamx`-backed) one, same shape as [`bevy_vector_to_parry`] but for the
/// integer voxel-grid-coordinate type `SharedShape::voxels`/`Voxels::new` take.
///
/// `f32`-precision only (`parry::math::IVector` is `glamx::IVec3`/`IVec2` under the `f32`
/// feature this repo builds with) -- the `f64` case uses a different parry-side `IVector`
/// width (`I64Vec3`/`I64Vec2`) that this repo's avian3d feature set (`f32`/`parry-f32`,
/// `apps/magicborn-kingdom/Cargo.toml`) never activates, so it is out of scope here.
#[inline]
#[must_use]
#[cfg(all(feature = "3d", feature = "f32"))]
pub(crate) fn bevy_ivector_to_parry(v: IVector) -> parry::math::IVector {
    parry::math::IVector::new(v.x, v.y, v.z)
}

/// See [`bevy_ivector_to_parry`]. 2D, `f32`-precision counterpart.
#[inline]
#[must_use]
#[cfg(all(feature = "2d", feature = "f32"))]
pub(crate) fn bevy_ivector_to_parry(v: IVector) -> parry::math::IVector {
    parry::math::IVector::new(v.x, v.y)
}

/// Bulk-converts a slice of Avian's own (bevy_math-backed) [`IVector`]s into a freshly
/// allocated `Vec` of `parry`-native (`glamx`-backed) grid-coordinate vectors. `f32`-precision
/// only -- see [`bevy_ivector_to_parry`].
#[inline]
#[must_use]
#[cfg(feature = "f32")]
pub(crate) fn bevy_ivector_slice_to_parry(v: &[IVector]) -> Vec<parry::math::IVector> {
    v.iter().copied().map(bevy_ivector_to_parry).collect()
}
