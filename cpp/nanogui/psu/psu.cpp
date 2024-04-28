#include "psu.h"

PSUScreen::PSUScreen(const Eigen::Vector2i &size, const std::string &caption, bool resizable)
    : nanogui::Screen(size, caption, resizable)
{
    m_size = size;
    setup();
}

PSUScreen::~PSUScreen() {

}

void PSUScreen::setup() {
    m_window = new nanogui::Window(this, "");
    m_layout = new nanogui::GridLayout();

    m_layout->setColAlignment({nanogui::Alignment::Maximum, nanogui::Alignment::Fill});
    m_layout->setSpacing(0, 16);
    m_layout->setSpacing(1, 0);
    m_layout->setMargin(10);

    m_window->setPosition(Eigen::Vector2i(0, 0));
    m_window->setLayout(m_layout);
    m_window->setFixedSize(m_size);
    
    {
	new nanogui::Label(m_window, "Battery Voltage", "sans", 18);
	m_voltage_tb = new nanogui::TextBox(m_window);
	m_voltage_tb->setEditable(true);
	m_voltage_tb->setValue("0");
	m_voltage_tb->setUnits("V");
	m_voltage_tb->setDefaultValue("0");
	m_voltage_tb->setFontSize(18);
	m_voltage_tb->setFormat("[-]?[0-9]*\\.?[0-9]+");
	m_voltage_tb->setFixedHeight(24);
    }
    {
	new nanogui::Label(m_window, "Battery Current", "sans", 18);
	m_current_tb = new nanogui::TextBox(m_window);
	m_current_tb->setEditable(true);
	m_current_tb->setValue("0");
	m_current_tb->setUnits("mA");
	m_current_tb->setDefaultValue("0");
	m_current_tb->setFontSize(18);
	m_current_tb->setFormat("[-]?[0-9]*\\.?[0-9]+");
	m_current_tb->setFixedHeight(24);
    }
    {
	new nanogui::Label(m_window, "Battery Failed?", "sans", 18);
	m_failure_cb = new nanogui::CheckBox(m_window, "");
	m_failure_cb->setChecked(false);
    }
    {
	new nanogui::Label(m_window, "On / Off", "sans", 18);
	m_on_off_cb = new nanogui::CheckBox(m_window, "");
	m_on_off_cb->setChecked(true);
    }
    {
	new nanogui::Label(m_window, "");
	m_send_btn = new nanogui::Button(m_window, "Send", ENTYPO_ICON_PUBLISH);
	m_send_btn->setCallback([=] {this->sendButton_onClicked();});	    
	m_send_btn->setFixedWidth(100);
	m_send_btn->setFixedHeight(24);
	m_send_btn->setFontSize(18);
    }
    
    this->setVisible(true);
    this->performLayout();
    
    return;
}


void PSUScreen::sendButton_onClicked(void) {
    std::cout << "Send Data:" << std::endl;
    std::cout << "Voltage : " << m_voltage_tb->value() << std::endl;
    std::cout << "Current : " << m_current_tb->value() << std::endl;
    std::cout << "Failure : "  << m_failure_cb->checked() << std::endl;
    std::cout << "On/Off  : "  << m_on_off_cb->checked() << std::endl;
    
    return;
}
