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

//#include <Common/Setup.h>
#include <Magnum/Magnum.h>
namespace Magnum { namespace Examples {
/****************************************************************************************************/
/* Common params */
extern const std::size_t g_FPS;
extern const std::size_t g_NumThreads;

/****************************************************************************************************/
/* Simulation params */
extern const bool  g_bIntegrationWithLineSearch;
extern const Float g_gravity;
extern const Float g_damping;
extern const Float g_dt;
extern const char* g_sceneName;

/****************************************************************************************************/
/* Material */
extern const char* g_materialType;
extern const Float g_mu;
extern const Float g_lambda;
extern const Float g_kappa;
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
extern const Float g_AnimationStartTime;
extern const Float g_AnimationDuration;

/****************************************************************************************************/
/* Collision */
extern const char* g_CollisionObjectType;
extern const bool  g_bRemoveContact;
extern const Float g_RemoveContactTime;
} }
