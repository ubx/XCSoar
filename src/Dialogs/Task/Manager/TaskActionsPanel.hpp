/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_TASK_ACTIONS_PANEL_HPP
#define XCSOAR_TASK_ACTIONS_PANEL_HPP

#include "Widget/RowFormWidget.hpp"
#include "Form/ActionListener.hpp"

class TaskManagerDialog;
class TaskMiscPanel;
class OrderedTask;

class TaskActionsPanel : public RowFormWidget, ActionListener {
  enum Controls {
    NEW_TASK,
    DECLARE,
    BROWSE,
    SAVE,
  };

  TaskManagerDialog &dialog;
  TaskMiscPanel &parent;

  OrderedTask **active_task;
  bool *task_modified;

public:
  TaskActionsPanel(TaskManagerDialog &_dialog, TaskMiscPanel &_parent,
                   OrderedTask **_active_task, bool *_task_modified);

private:
  void SaveTask();

  void OnBrowseClicked();
  void OnNewTaskClicked();
  void OnDeclareClicked();

  /* virtual methods from class Widget */
  virtual void Prepare(ContainerWindow &parent,
                       const PixelRect &rc) override;
  virtual void ReClick() override;
  virtual void Show(const PixelRect &rc) override;
  virtual void Hide() override;

  /* virtual methods from class ActionListener */
  virtual void OnAction(int id) override;
};

#endif
