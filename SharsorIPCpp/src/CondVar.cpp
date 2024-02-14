// Copyright (C) 2023  Andrea Patrizi (AndrePatri)
// 
// This file is part of SharsorIPCpp and distributed under the General Public License version 2 license.
// 
// SharsorIPCpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// SharsorIPCpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with SharsorIPCpp.  If not, see <http://www.gnu.org/licenses/>.

#include <SharsorIPCpp/CondVar.hpp>

namespace SharsorIPCpp {

    ConditionVariable::ConditionVariable(
                   std::string basename,
                   std::string name_space,
                   bool verbose,
                   VLevel vlevel)
        : _mem_config(basename, name_space),
        _basename(basename), _namespace(name_space),
        _verbose(verbose),
        _vlevel(vlevel),
        _journal(Journal(_getThisName()))
    {

    }

    std::string ConditionVariable::_getThisName()
    {

        return _this_name;
    }

}