/********************************************************************************
** Form generated from reading UI file 'QMain.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QMAIN_H
#define UI_QMAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QMain
{
public:
    QPushButton *GenerateYZModel;

    void setupUi(QWidget *QMain)
    {
        if (QMain->objectName().isEmpty())
            QMain->setObjectName(QString::fromUtf8("QMain"));
        QMain->resize(400, 300);
        GenerateYZModel = new QPushButton(QMain);
        GenerateYZModel->setObjectName(QString::fromUtf8("GenerateYZModel"));
        GenerateYZModel->setGeometry(QRect(20, 20, 111, 41));

        retranslateUi(QMain);
        QObject::connect(GenerateYZModel, SIGNAL(clicked()), GenerateYZModel, SLOT(click()));

        QMetaObject::connectSlotsByName(QMain);
    } // setupUi

    void retranslateUi(QWidget *QMain)
    {
        QMain->setWindowTitle(QApplication::translate("QMain", "QMain", nullptr));
        GenerateYZModel->setText(QApplication::translate("QMain", "\350\215\257\346\237\261\345\217\202\346\225\260\345\214\226\345\273\272\346\250\241", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QMain: public Ui_QMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QMAIN_H
