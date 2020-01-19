// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Input parameters
uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// Input computed in vertex shader
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

void main(void)
{
    // Apply bump mapping to normal vector to make waves look more detailed:
    vec4 bump = texture2D(bumpMap, bumpCoord)*2.0 - 1.0;
    vec3 N = normalize(rotMatrix * bump.xyz);

    // Reflected ray:
    vec3 E = normalize(eyeVec);
    vec3 R = reflect(E, N);
    // Gazebo requires rotated cube map lookup.
    R = vec3(R.x, R.z, R.y);

    // Get environment color of reflected ray:
    vec4 envColor = textureCube(cubeMap, R, 0.0);

	// Cheap hdr effect:
    envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

	// Compute refraction ratio (Fresnel):
    float facing = 1.0 - dot(-E, N);
    float refractionRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

    // Refracted ray only considers deep and shallow water colors:
    vec4 waterColor = mix(shallowColor, deepColor, facing);

    // Perform linear interpolation between reflection and refraction.
    vec4 color = mix(waterColor, envColor, refractionRatio);
    gl_FragColor = vec4(color.xyz, 0.9);
}
