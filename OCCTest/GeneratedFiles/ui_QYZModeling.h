/********************************************************************************
** Form generated from reading UI file 'QYZModeling.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QYZMODELING_H
#define UI_QYZMODELING_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QYZModeling
{
public:

    void setupUi(QWidget *QYZModeling)
    {
        if (QYZModeling->objectName().isEmpty())
            QYZModeling->setObjectName(QString::fromUtf8("QYZModeling"));
        QYZModeling->resize(400, 300);

        retranslateUi(QYZModeling);

        QMetaObject::connectSlotsByName(QYZModeling);
    } // setupUi

    void retranslateUi(QWidget *QYZModeling)
    {
        QYZModeling->setWindowTitle(QApplication::translate("QYZModeling", "QYZModeling", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QYZModeling: public Ui_QYZModeling {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QYZMODELING_H
