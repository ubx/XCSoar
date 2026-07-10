// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SystemDialog.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "UIGlobals.hpp"
#include "Language/Language.hpp"
#include "Widget/RowFormWidget.hpp"
#include "System.hpp"

class SystemWidget final
  : public RowFormWidget {

public:
  SystemWidget(const DialogLook &look):RowFormWidget(look) {}

private:
  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent,
               const PixelRect &rc) noexcept override;
};

void
SystemWidget::Prepare([[maybe_unused]] ContainerWindow &parent, [[maybe_unused]] const PixelRect &rc) noexcept
{
  AddButton(_("Reboot"), [](){ ColibriReboot(); });
}

void
ShowSystemDialog()
{
  const DialogLook &look = UIGlobals::GetDialogLook();
  TWidgetDialog<SystemWidget>
    dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(), look, _("System"));
  dialog.AddButton(_("Close"), mrOK);
  dialog.SetWidget(look);
  dialog.ShowModal();
}
