#!/usr/bin/env python3
from gpiozero import TonalBuzzer
from time import sleep

# Initialize a TonalBuzzer connected to GPIO18 (BCM)
tb = TonalBuzzer(18) 

# Define a musical tune as a sequence of notes and durations.
Default =	[
  ["E5",0.3],["Eb5",0.3],
  ["E5",0.3],["Eb5",0.3],["E5",0.3],["B4",0.3],["D5",0.3],["C5",0.3],
  ["A4",0.6],[None,0.1],["C4",0.3],["E4",0.3],["A4",0.3],
  ["B4",0.6],[None,0.1],["E4",0.3],["Ab4",0.3],["B4",0.3],
  ["C5",0.6],[None,0.1],["E4",0.3],["E5",0.3],["Eb5",0.3],
  ["E5",0.3],["Eb5",0.3],["E5",0.3],["B4",0.3],["D5",0.3],["C5",0.3],
  ["A4",0.6],[None,0.1],["C4",0.3],["E4",0.3],["A4",0.3],
  ["B4",0.6],[None,0.1],["E4",0.3],["C5",0.3],["B4",0.3],["A4",0.1]
  ]
Police =	[
    ["A4",0.3],["D5",0.3],["A4",0.3],["D5",0.3],["A4",0.3],["D5",0.3],
    ["A4",0.3],["D5",0.3],["A4",0.3],["D5",0.3],["A4",0.3],["D5",0.3]
    ]
    
URSS = [
    ["G4", 0.4],["C5", 0.8],["G4", 0.4],["A4", 0.4],["B4", 0.8],
    ["E4", 0.4],["E4", 0.4],["A4", 0.8],["G4", 0.4],["F4", 0.4],["G4", 0.8],
    ["C5", 0.8],["B4", 0.4],["A4", 0.4],["G4", 1.2],
]

Playlist = {
    "Default" : Default,
    "Police" : Police,
    "URSS" : URSS
    }
def play(tune):
    """
    Play a musical tune using the buzzer.
    :param tune: List of tuples (note, duration), 
    where each tuple represents a note and its duration.
    """
    for note, duration in tune:
        print(note)  # Output the current note being played
        tb.play(note)  # Play the note on the buzzer
        sleep(float(duration))  # Delay for the duration of the note
    tb.stop()  # Stop playing after the tune is complete

if __name__ == "__main__":
    try:
        play(Playlist[input("Choisir Musique : Default, Police, URSS ")])  # Execute the play function to start playing the tune.

    except KeyboardInterrupt:
        # Handle KeyboardInterrupt for graceful termination
        pass