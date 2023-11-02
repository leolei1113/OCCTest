/********************************************************************************
** Form generated from reading UI file 'interface.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_interfaceClass
{
public:
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *pushButton;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QPushButton *pushButton_2;
    QGridLayout *gridLayout;
    QLabel *label_3;
    QLineEdit *lineEdit_3;
    QLabel *label_4;
    QLineEdit *lineEdit_4;
    QPushButton *pushButton_3;

    void setupUi(QDialog *interfaceClass)
    {
        if (interfaceClass->objectName().isEmpty())
            interfaceClass->setObjectName(QString::fromUtf8("interfaceClass"));
        interfaceClass->resize(288, 245);
        gridLayout_2 = new QGridLayout(interfaceClass);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(interfaceClass);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(interfaceClass);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        horizontalLayout->addWidget(lineEdit);

        pushButton = new QPushButton(interfaceClass);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout->addWidget(pushButton);


        gridLayout_2->addLayout(horizontalLayout, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(interfaceClass);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        lineEdit_2 = new QLineEdit(interfaceClass);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        horizontalLayout_2->addWidget(lineEdit_2);

        pushButton_2 = new QPushButton(interfaceClass);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_2->addWidget(pushButton_2);


        gridLayout_2->addLayout(horizontalLayout_2, 1, 0, 1, 1);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_3 = new QLabel(interfaceClass);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 0, 1, 1);

        lineEdit_3 = new QLineEdit(interfaceClass);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        gridLayout->addWidget(lineEdit_3, 0, 1, 1, 2);

        label_4 = new QLabel(interfaceClass);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 2);

        lineEdit_4 = new QLineEdit(interfaceClass);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        gridLayout->addWidget(lineEdit_4, 1, 2, 1, 1);


        gridLayout_2->addLayout(gridLayout, 2, 0, 1, 1);

        pushButton_3 = new QPushButton(interfaceClass);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        gridLayout_2->addWidget(pushButton_3, 3, 0, 1, 1);


        retranslateUi(interfaceClass);

        QMetaObject::connectSlotsByName(interfaceClass);
    } // setupUi

    void retranslateUi(QDialog *interfaceClass)
    {
        interfaceClass->setWindowTitle(QApplication::translate("interfaceClass", "interface", nullptr));
        label->setText(QApplication::translate("interfaceClass", "\350\276\223\345\205\245\346\226\207\344\273\266", nullptr));
        pushButton->setText(QApplication::translate("interfaceClass", "\346\265\217\350\247\210", nullptr));
        label_2->setText(QApplication::translate("interfaceClass", "\350\276\223\345\207\272\346\226\207\344\273\266", nullptr));
        pushButton_2->setText(QApplication::translate("interfaceClass", "\346\265\217\350\247\210", nullptr));
        label_3->setText(QApplication::translate("interfaceClass", "\347\256\200\345\214\226\347\216\207", nullptr));
        label_4->setText(QApplication::translate("interfaceClass", "threshold\345\200\274", nullptr));
        pushButton_3->setText(QApplication::translate("interfaceClass", "\345\274\200\345\247\213\347\256\200\345\214\226", nullptr));
    } // retranslateUi

};

namespace Ui {
    class interfaceClass: public Ui_interfaceClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INTERFACE_H
