#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <string>
#include <vector>

std::string getProperty(Display* display, Window window, const std::string& propertyName) {
    Atom property = XInternAtom(display, propertyName.c_str(), False);
    Atom actualType;
    int actualFormat;
    unsigned long nitems, bytesAfter;
    unsigned char* prop = nullptr;

    int status = XGetWindowProperty(display, window, property, 0, (~0L), False, AnyPropertyType,
                                    &actualType, &actualFormat, &nitems, &bytesAfter, &prop);

    if (status != Success || prop == nullptr) {
        return "N/A";
    }

    std::string result;
    if (actualType == XA_STRING) {
        result = std::string(reinterpret_cast<char*>(prop));
    } else if (actualType == XInternAtom(display, "UTF8_STRING", False)) {
        result = std::string(reinterpret_cast<char*>(prop));
    } else {
        std::vector<unsigned long> values(reinterpret_cast<unsigned long*>(prop), reinterpret_cast<unsigned long*>(prop) + nitems);
        for (const auto& value : values) {
            result += std::to_string(value) + " ";
        }
    }

    XFree(prop);
    return result;
}

Window getWindowByPID(Display* display, pid_t pid) {
    Window root = DefaultRootWindow(display);
    Window parent;
    Window *children;
    unsigned int nchildren;

    if (XQueryTree(display, root, &root, &parent, &children, &nchildren) == 0) {
        return 0;
    }

    for (unsigned int i = 0; i < nchildren; i++) {
        Window w = children[i];
        Atom atom = XInternAtom(display, "_NET_WM_PID", true);
        Atom actual_type;
        int actual_format;
        unsigned long nitems, bytes_after;
        unsigned char *prop = nullptr;

        if (XGetWindowProperty(display, w, atom, 0, 1, false, XA_CARDINAL,
                               &actual_type, &actual_format, &nitems, &bytes_after, &prop) == Success && prop != nullptr) {
            if (pid == *((unsigned long *)prop)) {
                XFree(prop);
                return w;
            }
            XFree(prop);
        }
    }
    if (children) {
        XFree(children);
    }
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <PID> <Property>" << std::endl;
        return 1;
    }

    pid_t pid = std::stoi(argv[1]);
    std::string propertyName = argv[2];

    Display* display = XOpenDisplay(nullptr);
    if (display == nullptr) {
        std::cerr << "Cannot open display" << std::endl;
        return 1;
    }

    Window window = getWindowByPID(display, pid);
    if (window) {
        std::string propertyValue = getProperty(display, window, propertyName);
        std::cout << propertyName << ": " << propertyValue << std::endl;
    } else {
        std::cerr << "No window found with PID: " << pid << std::endl;
    }

    XCloseDisplay(display);
    return 0;
}
