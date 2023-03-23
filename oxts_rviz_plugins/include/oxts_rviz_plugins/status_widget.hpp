#pragma once

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QWidget>

#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "oxts_msgs/msg/ncom.hpp"
#include "oxts_rviz_plugins/NComRxC.h"
#include "oxts_rviz_plugins/QNavLabels.hpp"
#endif

namespace oxts_rviz_plugins {
class StatusWidget : public QWidget {
	Q_OBJECT
   public:
	StatusWidget(QWidget* parent = 0);
	~StatusWidget() override;

	rclcpp::Node::SharedPtr _node;

   public Q_SLOTS:
	void updateTopic();

   private:
	rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr subNcom_;
	std::string subTopic = "/ins/ncom";
	void ncomCallback(const oxts_msgs::msg::Ncom::SharedPtr msg);

	void initLayout();

	QLineEdit* TopicEditor;

	QNavStatusLabel* NavStatus;

	QTextLabel* TopicLabel;
	QTextLabel* NavLabel;
	QTextLabel* EastAccLabel;
	QTextLabel* NorthAccLabel;
	QTextLabel* UpAccLabel;
	QTextLabel* RollAccLabel;
	QTextLabel* PitchAccLabel;
	QTextLabel* YawAccLabel;
	QTextLabel* SatLabelPrimary;
	QTextLabel* SatLabelSecondary;

	QDoubleLabel* NorthAccStatus;
	QDoubleLabel* EastAccStatus;
	QDoubleLabel* UpAccStatus;
	QDoubleLabel* RollAccStatus;
	QDoubleLabel* PitchAccStatus;
	QDoubleLabel* YawAccStatus;

	QSatStatusLabel* SatStatusPrimary;
	QSatStatusLabel* SatStatusSecondary;

	std::vector<QInitializedLabel*> statusLabels;

	NComRxC* nrx;
};
}  // namespace oxts_rviz_plugins