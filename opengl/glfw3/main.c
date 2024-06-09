#include <stdio.h>
#include <GLFW/glfw3.h>

int main(int argc, char *argv) {
    GLFWwindow *window = NULL;
    
    if (GLFW_FALSE == glfwInit()) {
	perror("glfwInit");
	return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    
    if (NULL == (window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL))) {
	perror("glfwCreateWindow");
	glfwTerminate();
	return -1;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window)) {
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glfwSwapBuffers(window);
	glfwPollEvents();
    }
    
    glfwTerminate();

    puts("glfw done");
    
    return 0;
}
