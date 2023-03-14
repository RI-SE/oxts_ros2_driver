#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

#include <oxts_rviz_plugins/status_panel.hpp>


namespace oxts_rviz_plugins
{
    StatusPanel::StatusPanel(QWidget * parent) : Panel(parent)
    {
        QVBoxLayout * layout = new QVBoxLayout(this);
        _widget = new StatusWidget(parent);
        layout->addWidget(_widget);
        setLayout(layout);
    }

    StatusPanel::~StatusPanel()
    { }

    void StatusPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void StatusPanel::load(const rviz_common::Config &conf)
    {
        Panel::load(conf);
    }

    void StatusPanel::onInitialize()
    {
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        spinner_thread_ = std::make_shared<std::thread>(std::bind(&StatusPanel::spin, this));
    }

    void StatusPanel::spin() {
        while(rclcpp::ok()) {
            rclcpp::spin_some(_widget->_node);
        }
    }
} // namespace oxts_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(oxts_rviz_plugins::StatusPanel, rviz_common::Panel)
