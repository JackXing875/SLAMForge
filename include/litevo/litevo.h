// =============================================================================
// LiteVO — Industrial-Grade Monocular Visual SLAM System
// =============================================================================

#pragma once

/// @file litevo.h
/// @brief Single-include header for the LiteVO library.

// ── Version ──────────────────────────────────────────────────────────────────
#include "litevo/version.h"

// ── Base ─────────────────────────────────────────────────────────────────────
#include "litevo/base/exception.h"

// ── Core types ───────────────────────────────────────────────────────────────
#include "litevo/core/camera.h"
#include "litevo/core/config.h"
#include "litevo/core/feature_grid.h"
#include "litevo/core/frame.h"
#include "litevo/core/keyframe.h"
#include "litevo/core/map.h"
#include "litevo/core/map_point.h"
#include "litevo/core/types.h"

// ── Geometry ─────────────────────────────────────────────────────────────────
#include "litevo/geometry/epipolar.h"
#include "litevo/geometry/pnp.h"
#include "litevo/geometry/se3.h"
#include "litevo/geometry/sim3.h"
#include "litevo/geometry/triangulation.h"

// ── Features ─────────────────────────────────────────────────────────────────
#include "litevo/features/orb_extractor.h"

// ── Tracking ─────────────────────────────────────────────────────────────────
#include "litevo/tracking/initializer.h"
#include "litevo/tracking/matcher.h"
#include "litevo/tracking/tracker.h"

// ── Mapping ──────────────────────────────────────────────────────────────────
#include "litevo/mapping/local_mapper.h"

// ── Loop Closing ─────────────────────────────────────────────────────────────
#include "litevo/loop_closing/corrector.h"
#include "litevo/loop_closing/detector.h"
#include "litevo/loop_closing/global_ba.h"
#include "litevo/loop_closing/loop_closing.h"
#include "litevo/loop_closing/pose_graph.h"
#include "litevo/loop_closing/verifier.h"
#include "litevo/loop_closing/vocabulary.h"

// ── Optimization ─────────────────────────────────────────────────────────────
#include "litevo/optimization/ba.h"
