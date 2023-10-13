#version 410 core

#define NUM_LIGHTS 8

// vertex position in world coordinates
layout (location = 0) in vec3 position;
// vertex normal in world coordinate
layout (location = 1) in vec3 normal;
// UV coordinates
layout (location = 2) in vec2 UV;
// the numbers used for the location in the layout qualifier are the positions of the vertex attribute
// as defined in the Mesh class

// model matrix
uniform mat4 modelMatrix;
// view matrix
uniform mat4 viewMatrix;
// Projection matrix
uniform mat4 projectionMatrix;

// normals transformation matrix (= transpose of the inverse of the model-view matrix)
uniform mat3 normalMatrix;

// the position of the point light is passed as uniform
// N. B.) with more lights, and of different kinds, the shader code must be modified with a for cycle, with different treatment of the source lights parameters (directions, position, cutoff angle for spot lights, etc)
uniform vec3 pointLightPosition[NUM_LIGHTS];

// light incidence direction (in view coordinates)
out vec3 lightDir[NUM_LIGHTS];
// the transformed normal (in view coordinate) is set as an output variable, to be "passed" to the fragment shader
// this means that the normal values in each vertex will be interpolated on each fragment created during rasterization between two vertices
out vec3 vNormal;

// in the subroutines in fragment shader where specular reflection is considered,
// we need to calculate also the reflection vector for each fragment
// to do this, we need to calculate in the vertex shader the view direction (in view coordinates) for each vertex, and to have it interpolated for each fragment by the rasterization stage
out vec3 vViewPosition;

// vertex position and normal in world coordinates, to be calculated per vertex and interpolated for each fragment, in order to calculate the reflection vector in the fragment shader
out vec4 worldPosition;
out vec3 worldNormal;
out vec2 interp_UV;

out vec4 o_color;


void main(){

  // vertex position in ModelView coordinate (see the last line for the application of projection)
  // when I need to use coordinates in camera coordinates, I need to split the application of model and view transformations from the projection transformations
  vec4 mvPosition = viewMatrix * modelMatrix * vec4( position, 1.0 );

  // view direction, negated to have vector from the vertex to the camera
  vViewPosition = -mvPosition.xyz;

  // transformations are applied to the normal
  vNormal = normalize( normalMatrix * normal );

  // light incidence direction (in view coordinate)

  for (int i = 0; i<pointLightPosition.length(); i++)
  {
    vec4 lightPos = viewMatrix  * vec4(pointLightPosition[i], 1.0);
    lightDir[i] = lightPos.xyz - mvPosition.xyz;
  }

    // vertex position in world coordinate (= we apply only model trasformations)
  worldPosition = modelMatrix * vec4( position, 1.0 );

  // we calculate the normal in world coordinates: in this case we do not use the normalMatrix (= inverse of the transpose of the modelview matrix), but we use the inverse of the transpose of the model matrix only
  // We can think to pass this matrix as an uniform like the normalMatrix, if we do not want to calculate here
  worldNormal = inverse(transpose(mat3(modelMatrix))) * normal;

  // I assign the values to a variable with "out" qualifier so to use the per-fragment interpolated values in the Fragment shader
  interp_UV = UV;

  // we apply the projection transformation
  gl_Position = projectionMatrix * mvPosition;

}
