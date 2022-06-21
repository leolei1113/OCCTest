/********************************************************************************
** Form generated from reading UI file 'DrawModel.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DRAWMODEL_H
#define UI_DRAWMODEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>

QT_BEGIN_NAMESPACE

class Ui_DrawModel
{
public:

    void setupUi(QDialog *DrawModel)
    {
        if (DrawModel->objectName().isEmpty())
            DrawModel->setObjectName(QString::fromUtf8("DrawModel"));
        DrawModel->resize(400, 300);

        retranslateUi(DrawModel);

        QMetaObject::connectSlotsByName(DrawModel);
    } // setupUi

    void retranslateUi(QDialog *DrawModel)
    {
        DrawModel->setWindowTitle(QApplication::translate("DrawModel", "DrawModel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DrawModel: public Ui_DrawModel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRAWMODEL_H
