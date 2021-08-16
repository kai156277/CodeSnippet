#include "DefineExportProfileDialog.h"
#include "DataManager.h"
#include "HeaderAndFooterDialog.h"
#include "ValueFormattingDialog.h"
#include "VariableFormat.h"
#include <QtWidgets/qcombobox.h>
#include <QtWidgets/qlistwidget.h>
#include <QtWidgets/qmessagebox.h>

class DataVariableListWidgetItem : public QListWidgetItem
{
public:
    explicit DataVariableListWidgetItem(QListWidget *parent = nullptr, int type = QListWidgetItem::Type)
        : QListWidgetItem(parent, type)
    {
    }
    explicit DataVariableListWidgetItem(const QString &text, QListWidget *parent = nullptr, int type = QListWidgetItem::Type)
        : QListWidgetItem(text, parent, type)
    {
    }
    explicit DataVariableListWidgetItem(const QIcon &icon, const QString &text, QListWidget *view = nullptr, int type = Type)
        : QListWidgetItem(icon, text, view, type)
    {
    }
    DataVariableListWidgetItem(const DataVariableListWidgetItem &other)
        : QListWidgetItem(other)
        , source_name_(other.source_name_)
        , variable_name_(other.variable_name_)
        , format_(other.format_)
    {
    }

    pos::VariableFormat format() const { return format_; }
    QString             source_name() const { return source_name_; }
    QString             variable_name() const { return variable_name_; }

    void set_format(const pos::VariableFormat &format) { format_ = format; }
    void set_source_name(const QString &src_name) { source_name_ = src_name; }
    void set_variable_name(const QString &var_name) { variable_name_ = var_name; }

private:
    QString source_name_;
    QString variable_name_;

    pos::VariableFormat format_;
};

DefineExportProfileDialog::DefineExportProfileDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    //>>> source data
    connect(ui.source_variables_tabWidget,
            &QTabWidget::currentChanged,
            this,
            &DefineExportProfileDialog::CurrentDataSourceChanged);

    connect(ui.source_variable_add_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                SlotInsertSelectSourceVariableToExport(ui.export_variables_listWidget->count());
            });

    connect(ui.source_variable_insert_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                SlotInsertSelectSourceVariableToExport(ui.export_variables_listWidget->currentRow());
            });

    connect(ui.source_variable_info_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                auto list_item = CurrentSourceDataItem();
                auto item      = dynamic_cast<DataVariableListWidgetItem *>(list_item);
                if (item)
                {
                    VariableInfo(item->source_name(),
                                 item->variable_name());
                }
            });

    // >>> export data
    connect(ui.export_variable_remove_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                int index = ui.export_variables_listWidget->currentRow();
                if (index >= 0)
                    RemoveExportVariable(index);
            });

    connect(ui.export_variable_format_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                QListWidgetItem *list_item = ui.export_variables_listWidget->currentItem();
                SlotModifyExportVariableFormat(list_item);
            });

    connect(ui.export_variable_info_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                QListWidgetItem *list_item = ui.export_variables_listWidget->currentItem();
                auto             item      = dynamic_cast<DataVariableListWidgetItem *>(list_item);
                if (item)
                {
                    VariableInfo(item->source_name(),
                                 item->variable_name());
                }
            });

    connect(ui.export_variable_up_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                int index = ui.export_variables_listWidget->currentRow();
                if (index >= 0)
                    UpExportVariable(index);
            });

    connect(ui.export_variable_down_pushButton,
            &QPushButton::clicked,
            [=](bool) {
                int index = ui.export_variables_listWidget->currentRow();
                if (index >= 0)
                    DownExportVariable(index);
            });
    connect(ui.export_variables_listWidget,
            &QListWidget::itemDoubleClicked,
            [=](QListWidgetItem *list_item) {
                SlotModifyExportVariableFormat(list_item);
            });

    // >>> other push button
    connect(ui.header_footer_pushButton,
            &QPushButton::clicked,
            this,
            &DefineExportProfileDialog::SlotDisplayHeaderAndFooterDialog);
}

void DefineExportProfileDialog::AddDataSource(const QString &src_name)
{
    if (auto sptr = pos::DataManager::Instance()->GetDataSource(src_name).lock())
    {
        QListWidget *listWidget = new QListWidget;
        for (const auto &variable : sptr->SupportVariables())
        {
            QString display = sptr->HumanReadableVariable(variable);
            auto    item    = new DataVariableListWidgetItem(display);
            item->set_source_name(src_name);
            item->set_variable_name(variable);
            listWidget->addItem(item);
        }
        ui.source_variables_tabWidget->addTab(listWidget, sptr->Name());
        connect(listWidget,
                &QListWidget::itemDoubleClicked,
                this,
                &DefineExportProfileDialog::SlotDoubleClickSourceItem);
    }
}

void DefineExportProfileDialog::SetFileExtensionName(const QString &name)
{
    ui.file_extension_lineEdit->setText(name);
}

QListWidgetItem *DefineExportProfileDialog::CurrentSourceDataItem()
{
    int index = ui.source_variables_tabWidget->currentIndex();
    if (index >= 0)
    {
        QWidget *    widget     = ui.source_variables_tabWidget->widget(index);
        QListWidget *listWidget = qobject_cast<QListWidget *>(widget);
        if (listWidget)
        {
            return listWidget->currentItem();
        }
    }
    return nullptr;
}

void DefineExportProfileDialog::CurrentDataSourceChanged(int index)
{
    printf("%d", index);
}

void DefineExportProfileDialog::InsertSourceVariableToExport(const QString &            src_name,
                                                             const QString &            variable_name,
                                                             const pos::VariableFormat &format,
                                                             int                        insert_index)
{
    if (auto sptr = pos::DataManager::Instance()->GetDataSource(src_name).lock())
    {
        auto display = sptr->HumanReadableVariable(variable_name);

        QString newText = QString("%1 [%2]")
                              .arg(display)
                              .arg(pos::toHumanString(format.unit()));
        auto item = new DataVariableListWidgetItem(newText);
        item->set_source_name(src_name);
        item->set_variable_name(variable_name);
        item->set_format(format);
        ui.export_variables_listWidget->insertItem(insert_index, item);
    }
}

void DefineExportProfileDialog::VariableInfo(const QString &src_name, const QString &variable_name)
{
    QMessageBox::information(this, QString::fromUtf8("变量信息"), variable_name);
}

void DefineExportProfileDialog::RemoveExportVariable(int index)
{
    QListWidgetItem *item = ui.export_variables_listWidget->takeItem(index);
    delete item;
}

void DefineExportProfileDialog::UpExportVariable(int index)
{
    if (index == 0)
        return;

    int target = index - 1;

    auto item = ui.export_variables_listWidget->takeItem(index);
    ui.export_variables_listWidget->insertItem(target, item);
    ui.export_variables_listWidget->setCurrentRow(target);
}

void DefineExportProfileDialog::DownExportVariable(int index)
{
    if (index == (ui.export_variables_listWidget->count() - 1))
        return;
    int target = index + 1;

    auto item = ui.export_variables_listWidget->takeItem(index);
    ui.export_variables_listWidget->insertItem(target, item);
    ui.export_variables_listWidget->setCurrentRow(target);
}

void DefineExportProfileDialog::SetHeaderAndFooter(const HeaderAndFooterProfile &profile)
{
    header_footer_ = profile;
}

void DefineExportProfileDialog::SlotInsertSelectSourceVariableToExport(int export_index)
{
    auto list_item = CurrentSourceDataItem();
    auto item      = dynamic_cast<DataVariableListWidgetItem *>(list_item);
    if (item)
    {
        QString src_name = item->source_name();
        QString var_name = item->variable_name();

        ValueFormattingDialog dialog;
        dialog.SetValueFormat(src_name, var_name, item->format());
        if (dialog.exec() == QDialog::Accepted)
        {
            InsertSourceVariableToExport(src_name, var_name, dialog.format(), export_index);
        }
    }
}

void DefineExportProfileDialog::SlotDoubleClickSourceItem(QListWidgetItem *list_item)
{
    auto item = dynamic_cast<DataVariableListWidgetItem *>(list_item);
    if (item)
    {
        QString src_name = item->source_name();
        QString var_name = item->variable_name();

        ValueFormattingDialog dialog;
        dialog.SetValueFormat(src_name, var_name, item->format());
        if (dialog.exec() == QDialog::Accepted)
        {
            int export_index = ui.export_variables_listWidget->count();
            InsertSourceVariableToExport(src_name, var_name, dialog.format(), export_index);
        }
    }
}

void DefineExportProfileDialog::SlotModifyExportVariableFormat(QListWidgetItem *list_item)
{
    auto item = dynamic_cast<DataVariableListWidgetItem *>(list_item);
    if (item)
    {
        ValueFormattingDialog dialog;

        QString src_name = item->source_name();
        QString var_name = item->variable_name();

        dialog.SetValueFormat(src_name, var_name, item->format());
        if (dialog.exec() == QDialog::Accepted)
        {
            item->set_format(dialog.format());
            QString newText = QString("%1 [%2]")
                                  .arg(dialog.variable_human_name())
                                  .arg(pos::toHumanString(dialog.format().unit()));
            item->setText(newText);
        }
    }
}

void DefineExportProfileDialog::SlotDisplayHeaderAndFooterDialog(bool)
{
    HeaderAndFooterDialog dialog;
    dialog.set_profile(header_footer_);
    if (dialog.exec() == QDialog::Accepted)
    {
        header_footer_ = dialog.profile();
    }
}
