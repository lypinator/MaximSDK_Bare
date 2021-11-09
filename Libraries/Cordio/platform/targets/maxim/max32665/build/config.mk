###################################################################################################
#
# CHIP build options
#
# Copyright (c) 2019-2020 Packetcraft, Inc.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
###################################################################################################

CFG_DEV         += HCI_UART=2
CFG_DEV         += TERMINAL_UART=1
CFG_DEV         += USER_UART=0

CFG_DEV 				+= HCI_UART_MAP=MAP_A
CFG_DEV 				+= TERMINAL_UART_MAP=MAP_A
CFG_DEV 				+= USER_UART_MAP=MAP_A

CFG_DEV 				+= HCI_UART_MAP=MAP_A
CFG_DEV 				+= TERMINAL_UART_MAP=MAP_A
CFG_DEV 				+= USER_UART_MAP=MAP_A
