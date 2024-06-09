#include <stdio.h>
#include <GLFW/glfw3.h>

void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void drawTriangle();

int main(int argc, char *argv) {
    GLFWwindow *window = NULL;
    
    if (GLFW_FALSE == glfwInit()) {
	perror("glfwInit");
	return -1;
    }

    glfwSetErrorCallback(error_callback);   
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    
    if (NULL == (window = glfwCreateWindow(640, 480, "Triangle", NULL, NULL))) {
	perror("glfwCreateWindow");
	glfwTerminate();
	return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    glViewport(0, 0, 640, 480);
    
    while (!glfwWindowShouldClose(window)) {
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	drawTriangle();
	glfwSwapBuffers(window);
	glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
   
    return 0;
}

void error_callback(int error, const char* description) {
    printf("error: %s\n", description);
    return;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    return;
}

void drawTriangle() {
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex2f(-0.5f, -0.5f);
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glVertex2f(0.5f, -0.5f);
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glVertex2f(0.0f, 0.5f);
    glEnd();
    
    return;
}
