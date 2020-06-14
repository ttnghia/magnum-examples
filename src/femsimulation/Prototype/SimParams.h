/**
 * Copyright 2019 Nghia Truong <nghiatruong.vn@gmail.com>
 *                Kui Wu <walker.kui.wu@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <Common/Setup.h>

/****************************************************************************************************/
/* Common params */
extern const u64 g_FPS;
extern const u64 g_NumThreads;

/****************************************************************************************************/
/* Simulation params */
extern const bool  g_bIntegrationWithLineSearch;
extern const real  g_gravity;
extern const real  g_damping;
extern const real  g_dt;
extern const char* g_sceneName;

/****************************************************************************************************/
/* Material */
extern const char* g_materialType;
extern const real  g_mu;
extern const real  g_lambda;
extern const real  g_kappa;
extern const bool  g_bAllowSceneToChangeMaterialStiffnesses;

/****************************************************************************************************/
/* Inverse simulation */
extern const bool   g_bInverseSimulation;
extern const bool   g_bPrintDebugDetails;
extern const bool   g_bCrashIfFailed;
extern const double g_ForceRegularization;
extern const double g_TorqueRegularization;

/****************************************************************************************************/
/* Animation */
extern const char* g_AnimationType;
extern const bool  g_bAnimation;
extern const real  g_AnimationStartTime;
extern const real  g_AnimationDuration;

/****************************************************************************************************/
/* Collision */
extern const char* g_CollisionObjectType;
extern const bool  g_bRemoveContact;
extern const real  g_RemoveContactTime;
