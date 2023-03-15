#include <oxts_rviz_plugins/status_widget.hpp>

namespace oxts_rviz_plugins {

StatusWidget::StatusWidget(QWidget* parent) : QWidget(parent) {
	auto options = rclcpp::NodeOptions();
	_node = std::make_shared<rclcpp::Node>("oxts_plugin", options);
	subNcom_ = _node->create_subscription<oxts_msgs::msg::Ncom>(
		"/ins/ncom", 1, std::bind(&StatusWidget::ncomCallback, this, std::placeholders::_1));

	RCLCPP_INFO(_node->get_logger(), "Subscribing to /ins/ncom");

    initLayout();

	nrx = NComCreateNComRxC();
}

StatusWidget::~StatusWidget() {
	delete nrx;
}

void StatusWidget::initLayout() {
    NavLabel = new QTextLabel(this, "Navigation system mode:");
	EastAccLabel = new QTextLabel(this, "East accuracy:");
	NorthAccLabel = new QTextLabel(this, "North accuracy:");
	UpAccLabel = new QTextLabel(this, "Up accuracy:");
    RollAccLabel = new QTextLabel(this, "Roll accuracy:");
    PitchAccLabel = new QTextLabel(this, "Pitch accuracy:");
    YawAccLabel = new QTextLabel(this, "Yaw accuracy:");
    SatLabelPrimary = new QTextLabel(this, "Primary antenna #satellites tracked:");
    SatLabelSecondary = new QTextLabel(this, "Secondary antenna #satellites tracked:");

	NorthAccStatus = new QDoubleLabel(this);
	EastAccStatus = new QDoubleLabel(this);
	UpAccStatus = new QDoubleLabel(this);
    RollAccStatus = new QDoubleLabel(this);
    PitchAccStatus = new QDoubleLabel(this);
    YawAccStatus = new QDoubleLabel(this);

    SatStatusPrimary = new QSatStatusLabel(this);
    SatStatusSecondary = new QSatStatusLabel(this);

	NavStatus = new QNavStatusLabel(this);

    RollAccStatus->setLimits(0.2, 0.5);
    PitchAccStatus->setLimits(0.2, 0.5);
    YawAccStatus->setLimits(0.2, 0.5);

	QGridLayout* grid = new QGridLayout(this);

	grid->addWidget(NavLabel, 0, 0);
	grid->addWidget(NavStatus, 0, 1);
	grid->addWidget(EastAccLabel, 1, 0);
	grid->addWidget(EastAccStatus, 1, 1);
	grid->addWidget(NorthAccLabel, 2, 0);
	grid->addWidget(NorthAccStatus, 2, 1);
	grid->addWidget(UpAccLabel, 3, 0);
	grid->addWidget(UpAccStatus, 3, 1);
    grid->addWidget(RollAccLabel, 4, 0);
    grid->addWidget(RollAccStatus, 4, 1);
    grid->addWidget(PitchAccLabel, 5, 0);
    grid->addWidget(PitchAccStatus, 5, 1);
    grid->addWidget(YawAccLabel, 6, 0);
    grid->addWidget(YawAccStatus, 6, 1);
    grid->addWidget(SatLabelPrimary, 7, 0);
    grid->addWidget(SatStatusPrimary, 7, 1);
    grid->addWidget(SatLabelSecondary, 8, 0);
    grid->addWidget(SatStatusSecondary, 8, 1);

	setLayout(grid);
}

void StatusWidget::ncomCallback(const oxts_msgs::msg::Ncom::SharedPtr msg) {
	if (NComNewChars(this->nrx, msg->raw_packet.data(), NCOM_PACKET_LENGTH) == COM_NEW_UPDATE) {
		if(this->nrx->mIsEastAccValid)
            EastAccStatus->setValue(this->nrx->mEastAcc);
		if(this->nrx->mIsNorthAccValid)
            NorthAccStatus->setValue(this->nrx->mNorthAcc);
		if(this->nrx->mIsAltAccValid)
            UpAccStatus->setValue(this->nrx->mAltAcc);
        if(this->nrx->mIsRollAccValid)
            RollAccStatus->setValue(this->nrx->mRollAcc);
        if(this->nrx->mIsPitchAccValid)
            PitchAccStatus->setValue(this->nrx->mPitchAcc);
        if(this->nrx->mIsHeadingAccValid)
            YawAccStatus->setValue(this->nrx->mHeadingAcc);
        if(this->nrx->mIsInsNavModeValid)
		    NavStatus->setValue(this->nrx->mInsNavMode);
        if(this->nrx->mGpsPrimary->mIsNumSatsValid)
            SatStatusPrimary->setValue(this->nrx->mGpsPrimary->mNumSats);
        if(this->nrx->mGpsSecondary->mIsNumSatsValid)
            SatStatusSecondary->setValue(this->nrx->mGpsSecondary->mNumSats);
	}
}
}  // namespace oxts_rviz_plugins
