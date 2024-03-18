#!/bin/bash
# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.


# Starte eine neue Byobu-Session im Detached-Modus
byobu new-session -d -s mysession

# Führe das erste Python-Skript in der ersten Pane aus
byobu send-keys -t mysession:0 'clear && for i in {1..3}; do python3 _test_manual_server.py; done' C-m

# Erstelle eine neue horizontale Split-Pane
byobu split-window -h

# Führe das zweite Python-Skript in der neuen Pane aus
byobu send-keys -t mysession:0.1 'clear && for i in {1..10}; do python3 _test_manual_client.py; done' C-m

# Anhängen der Byobu-Session, um sie sichtbar zu machen
byobu attach-session -t mysession

