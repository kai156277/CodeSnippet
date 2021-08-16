#pragma once

#include "HeaderAndFooterDialog.h"
#include "VariableFormat.h"
#include "ui_DefineExportProfileDialog.h"
#include <QtWidgets/qdialog.h>

class DefineExportProfileDialog : public QDialog
{
    Q_OBJECT
public:
    DefineExportProfileDialog(QWidget *parent = nullptr);

    void AddDataSource(const QString &src_name);
    // 不需要加.
    void SetFileExtensionName(const QString &name);

    void InsertSourceVariableToExport(const QString &            src_name,
                                      const QString &            variable_name,
                                      const pos::VariableFormat &format,
                                      int                        insert_index);
    void VariableInfo(const QString &src_name, const QString &variable_name);

    void RemoveExportVariable(int index);
    void UpExportVariable(int index);
    void DownExportVariable(int index);

    void SetHeaderAndFooter(const HeaderAndFooterProfile &profile);

private:
    QListWidgetItem *CurrentSourceDataItem();
    void             CurrentDataSourceChanged(int index);
    void             SlotInsertSelectSourceVariableToExport(int export_index);
    void             SlotDoubleClickSourceItem(QListWidgetItem *list_item);
    void             SlotModifyExportVariableFormat(QListWidgetItem *list_item);
    void             SlotDisplayHeaderAndFooterDialog(bool);

    Ui::DefineExportProfileDialog ui;
    HeaderAndFooterProfile        header_footer_;
};
