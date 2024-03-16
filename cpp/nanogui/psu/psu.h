#ifndef PSU_H
#define PSU_H

#include "nanogui/nanogui.h"

class PSUScreen : public nanogui::Screen {
public:
    PSUScreen(const Eigen::Vector2i &size, const std::string &caption, bool resizable = true);
    virtual ~PSUScreen();
    
private:
    Eigen::Vector2i m_size;
    nanogui::Window *m_window = nullptr;
    nanogui::GridLayout *m_layout = nullptr;
    nanogui::TextBox *m_voltage_tb = nullptr;
    nanogui::TextBox *m_current_tb = nullptr;
    nanogui::CheckBox *m_failure_cb = nullptr;
    nanogui::CheckBox *m_on_off_cb = nullptr;
    nanogui::Button *m_send_btn = nullptr;

    void setup();
    /// function that is callback registered to send button
    void sendButton_onClicked(void);
};

#endif
