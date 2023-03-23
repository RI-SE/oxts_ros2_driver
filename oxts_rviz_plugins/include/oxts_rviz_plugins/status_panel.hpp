#pragma once

#include <QLabel>
#include <QVBoxLayout>

#include <oxts_rviz_plugins/status_widget.hpp>
#include <rviz_common/panel.hpp>

namespace oxts_rviz_plugins {
class StatusWidget;

class StatusPanel : public rviz_common::Panel {
	Q_OBJECT
   public:
	explicit StatusPanel(QWidget* parent = 0);
	virtual ~StatusPanel();

	void onInitialize() override;
	void save(rviz_common::Config config) const override;
	void load(const rviz_common::Config& conf) override;

   private:
	void spin();

	StatusWidget* _widget;
	std::shared_ptr<std::thread> spinner_thread_;
};
}  // namespace oxts_rviz_plugins
