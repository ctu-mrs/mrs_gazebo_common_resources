/**  \file
     \brief Defines class that will modify the the Gazebo GUI after its running.
     \author Viktor Walter - viktor.walter@fel.cvut.cz (original implementation)
 */

// Include the headers
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>

namespace gazebo
{
class GuiManager : public GUIPlugin {

public:
  void Load([[maybe_unused]] sdf::ElementPtr sdf) {

    std::cout << "[GuiManager]: Starting plugin" << std::endl;

    this->resize(0, 0);

    gui::Events::leftPaneVisibility.Signal(false);
    /* gui::Events::showToolbars.Signal(false); */
    /* gui::Events::fullScreen.Signal(true); */

    std::cout << "[GuiManager]: Finished" << std::endl;
  }
};

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GuiManager)
}  // namespace gazebo
