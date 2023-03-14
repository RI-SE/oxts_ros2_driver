#include <oxts_rviz_plugins/status_widget.hpp>

namespace oxts_rviz_plugins {

StatusWidget::StatusWidget(QWidget* parent) : QWidget(parent) {
	auto options = rclcpp::NodeOptions();
	_node = std::make_shared<rclcpp::Node>("oxts_plugin", options);
	subNcom_ = _node->create_subscription<oxts_msgs::msg::Ncom>(
		"/ins/ncom", 1, std::bind(&StatusWidget::ncomCallback, this, std::placeholders::_1));

	RCLCPP_INFO(_node->get_logger(), "Subscribing to /ins/ncom");

    NavLabel = new QLabel(this);
    NavStatus = new QDoubleLabel(this);
	EastAccLabel = new QLabel(this);
    EastAccStatus = new QDoubleLabel(this);
	NorthAccLabel = new QLabel(this);
    NorthAccStatus = new QDoubleLabel(this);
    UpAccLabel = new QLabel(this);
    UpAccStatus = new QDoubleLabel(this);

    NavLabel->setText("Navigation system mode:");
	NavLabel->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	NavLabel->setWordWrap(true);

    EastAccLabel->setText("East accuracy:");
	EastAccLabel->setToolTip("East Accuracy of NCOM");
	EastAccLabel->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	EastAccLabel->setWordWrap(true);

	NorthAccLabel->setText("North accuracy:");
	NorthAccLabel->setToolTip("North Accuracy of NCOM");
	NorthAccLabel->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	NorthAccLabel->setWordWrap(true);

	UpAccLabel->setText("Up accuracy:");
	UpAccLabel->setToolTip("Up Accuracy of NCOM");
	UpAccLabel->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	UpAccLabel->setWordWrap(true);

    
    QGridLayout* grid = new QGridLayout(this);

    grid->addWidget(NavLabel, 0, 0);
    grid->addWidget(NavStatus, 0, 1);
    grid->addWidget(EastAccLabel, 1, 0);
    grid->addWidget(EastAccStatus, 1, 1);
    grid->addWidget(NorthAccLabel, 2, 0);
    grid->addWidget(NorthAccStatus, 2, 1);
    grid->addWidget(UpAccLabel, 3, 0);
    grid->addWidget(UpAccStatus, 3, 1);

	setLayout(grid);

	nrx = NComCreateNComRxC();
}

StatusWidget::~StatusWidget() {
	delete nrx;
}

void StatusWidget::ncomCallback(const oxts_msgs::msg::Ncom::SharedPtr msg) {
	if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) == COM_NEW_UPDATE) {
        uint8_t navMode = this->nrx->mInsNavMode;

        EastAccStatus->setValue(this->nrx->mEastAcc);
        NorthAccStatus->setValue(this->nrx->mNorthAcc);
        UpAccStatus->setValue(this->nrx->mAltAcc);
        
        NavStatus->setText(QString::number(navMode));
	}
}


}  // namespace oxts_rviz_plugins
