// =============================================================================
// SLAMForge — Monocular Visual SLAM and Dense Reconstruction
// =============================================================================

#pragma once

/// @file slamforge.h
/// @brief Single-include header for the SLAMForge library.

// ── Version ──────────────────────────────────────────────────────────────────
#include "slamforge/version.h"

// ── Base ─────────────────────────────────────────────────────────────────────
#include "slamforge/base/exception.h"

// ── Core types ───────────────────────────────────────────────────────────────
#include "slamforge/core/camera.h"
#include "slamforge/core/config.h"
#include "slamforge/core/feature_grid.h"
#include "slamforge/core/frame.h"
#include "slamforge/core/keyframe.h"
#include "slamforge/core/map.h"
#include "slamforge/core/map_point.h"
#include "slamforge/core/types.h"

// ── Geometry ─────────────────────────────────────────────────────────────────
#include "slamforge/geometry/epipolar.h"
#include "slamforge/geometry/pnp.h"
#include "slamforge/geometry/se3.h"
#include "slamforge/geometry/sim3.h"
#include "slamforge/geometry/triangulation.h"

// ── Features ─────────────────────────────────────────────────────────────────
#include "slamforge/features/orb_extractor.h"

// ── Input / Output ───────────────────────────────────────────────────────────
#include "slamforge/io/map_export.h"

// ── Tracking ─────────────────────────────────────────────────────────────────
#include "slamforge/tracking/initializer.h"
#include "slamforge/tracking/matcher.h"
#include "slamforge/tracking/tracker.h"

// ── Mapping ──────────────────────────────────────────────────────────────────
#include "slamforge/mapping/dense_mapper.h"
#include "slamforge/mapping/local_mapper.h"

// ── Loop Closing ─────────────────────────────────────────────────────────────
#include "slamforge/loop_closing/corrector.h"
#include "slamforge/loop_closing/detector.h"
#include "slamforge/loop_closing/global_ba.h"
#include "slamforge/loop_closing/loop_closing.h"
#include "slamforge/loop_closing/pose_graph.h"
#include "slamforge/loop_closing/verifier.h"
#include "slamforge/loop_closing/vocabulary.h"

// ── Optimization ─────────────────────────────────────────────────────────────
#include "slamforge/optimization/ba.h"
