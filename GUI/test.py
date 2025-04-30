import glfw
from OpenGL.GL import *
import numpy as np
import ctypes
import math
from pyrr import Matrix44, Vector3

# Shaders
VERTEX_SHADER = """
#version 330 core
layout (location = 0) in vec3 position;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main()
{
    gl_Position = projection * view * model * vec4(position, 1.0);
}
"""


FRAGMENT_SHADER = """#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(0.2, 0.7, 0.3, 1.0);
}"""

def compile_shader(source, shader_type):
    shader = glCreateShader(shader_type)
    glShaderSource(shader, source)
    glCompileShader(shader)
    if not glGetShaderiv(shader, GL_COMPILE_STATUS):
        raise RuntimeError(glGetShaderInfoLog(shader))
    return shader

def create_shader_program(vertex_src, fragment_src):
    vertex_shader = compile_shader(vertex_src, GL_VERTEX_SHADER)
    fragment_shader = compile_shader(fragment_src, GL_FRAGMENT_SHADER)
    program = glCreateProgram()
    glAttachShader(program, vertex_shader)
    glAttachShader(program, fragment_shader)
    glLinkProgram(program)
    if not glGetProgramiv(program, GL_LINK_STATUS):
        raise RuntimeError(glGetProgramInfoLog(program))
    glDeleteShader(vertex_shader)
    glDeleteShader(fragment_shader)
    return program

def main():
    # Initialize window
    if not glfw.init():
        return
    window = glfw.create_window(800, 600, "3D Cube", None, None)
    glfw.make_context_current(window)

    # Define cube vertices
    vertices = np.array([
        -0.5, -0.5, -0.5,
         0.5, -0.5, -0.5,
         0.5,  0.5, -0.5,
        -0.5,  0.5, -0.5,
        -0.5, -0.5,  0.5,
         0.5, -0.5,  0.5,
         0.5,  0.5,  0.5,
        -0.5,  0.5,  0.5,
    ], dtype=np.float32)

    indices = np.array([
        0, 1, 2, 2, 3, 0,  # back face
        4, 5, 6, 6, 7, 4,  # front face
        0, 1, 5, 5, 4, 0,  # bottom
        2, 3, 7, 7, 6, 2,  # top
        1, 2, 6, 6, 5, 1,  # right
        3, 0, 4, 4, 7, 3,  # left
    ], dtype=np.uint32)

    VAO = glGenVertexArrays(1)
    VBO = glGenBuffers(1)
    EBO = glGenBuffers(1)

    glBindVertexArray(VAO)

    glBindBuffer(GL_ARRAY_BUFFER, VBO)
    glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
    glEnableVertexAttribArray(0)

    # Compile and use shader
    shader = create_shader_program(VERTEX_SHADER, FRAGMENT_SHADER)
    glUseProgram(shader)

    glEnable(GL_DEPTH_TEST)

        # Set up projection
    proj = Matrix44.perspective_projection(
        fovy=45.0, aspect=800/600, near=0.1, far=100.0
    )
    proj_loc = glGetUniformLocation(shader, "projection")
    glUniformMatrix4fv(proj_loc, 1, GL_FALSE, proj.astype('float32'))

    while not glfw.window_should_close(window):
        glfw.poll_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Model: Rotate around Y
        angle = glfw.get_time()
        model = Matrix44.from_y_rotation(angle)

        # View: Camera back and up
        eye = Vector3([2.0, 2.0, 2.0])
        target = Vector3([0.0, 0.0, 0.0])
        up = Vector3([0.0, 1.0, 0.0])
        view = Matrix44.look_at(eye, target, up)

        # Send matrices to shader
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, model.astype('float32'))
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, view.astype('float32'))

        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES, len(indices), GL_UNSIGNED_INT, None)
        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()
