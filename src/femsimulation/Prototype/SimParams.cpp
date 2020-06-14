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

#include <SimParams.h>
namespace Magnum { namespace Examples {
/****************************************************************************************************/
/* Common params */
const std::size_t g_FPS        = 30; /* target 30 frame per second */
const std::size_t g_NumThreads = 0;  /* numThreads = 0 mean using all threads */

/****************************************************************************************************/
/* Simulation params */
const bool  g_bIntegrationWithLineSearch = true;
const Float g_gravity = 10;
const Float g_damping = Float(5e-3);
const Float g_dt      = Float(1e-3);

/****************************************************************************************************/
/* Scene names */
const char* g_sceneName = "beam";
//const char* g_sceneName = "two-elements";
//const char* g_sceneName = "one-element";

/****************************************************************************************************/
/* Material params
 * Stiffnesses can be override during scene initialization
 */
const char* g_materialType = "neo";
//const char* g_materialType = "stvk";
const bool g_bAllowSceneToChangeMaterialStiffnesses = true;

const Float g_lambda = 25;
const Float g_kappa  = 200;
const Float g_mu     = 1000; /* can be overriden in Scene classes */

/****************************************************************************************************/
/* Inverse simulation */
const bool g_bInverseSimulation = true;

const bool   g_bPrintDebugDetails   = true;
const bool   g_bCrashIfFailed       = false;
const double g_ForceRegularization  = 1e-5;
const double g_TorqueRegularization = 1; /* value < 0 mean hard constraint */

/****************************************************************************************************/
/* Animation */
const char* g_AnimationType      = "down-up";
const bool  g_bAnimation         = true;
const Float g_AnimationStartTime = Float(6); /* start after number of secs */
const Float g_AnimationDuration  = Float(1); /* duration in secs */

/****************************************************************************************************/
/* Collision */
const char* g_CollisionObjectType = "rectangle";
//const char* g_CollisionObjectType = "nope";
const bool  g_bRemoveContact    = true;
const Float g_RemoveContactTime = 3; /* time to remove collision object */
} }
