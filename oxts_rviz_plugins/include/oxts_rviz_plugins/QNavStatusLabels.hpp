#pragma once

#include <QLabel>
#include <QWidget>

struct Limit {
    double green = 0.05;
    double yellow = 0.1;
    double red = __DBL_MAX__;
};

class QDoubleLabel : public QLabel {
	Q_OBJECT
    
    enum class Status {
        OK,
        WARNING,
        ERROR
    };

   public:
	explicit QDoubleLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags()) {
        QLabel(parent, f);

        this->setText("Awaiting data");
        this->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
	virtual ~QDoubleLabel() {};

    void setValue(const double d_text) {
        this->setText(QString::number(d_text, 'f', 3));
        updateStyle(d_text);        
    }

   private:
    void updateStyle(double d_text) {
        auto newStatus = getStatus(d_text);
        if(status == newStatus) {
            return;
        }

        status = newStatus;
        if(status == Status::OK) {
            this->setStyleSheet("background-color : green;");
        } else if(status == Status::WARNING) {
            this->setStyleSheet("background-color : yellow;");
        } else if(status == Status::ERROR) {
            this->setStyleSheet("background-color : red;");
        }
    }

    Status getStatus(const double d) {
        if(d < limit.green) {
            return Status::OK;
        } else if (d >= limit.green && d < limit.yellow) {
            return Status::WARNING;
        } else {
            return Status::ERROR;
        }
    }

    Status status = Status::ERROR;
    Limit limit;
};