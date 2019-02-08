import numpy
# scipy. special fuer sigmoid function expit()
import scipy.special                                    # cmd: 'sudo apt-get install python-scipy python-matplotlib'
# Bibliothek zur Darstellung der Array
import matplotlib.pyplot
#      %matplotlib inline


# neural network class definition
class neuralNetwork:
    
    # initialise the neural network
    def __init__(self, inputnodes, hiddennodes, outputnodes, learningrate):
        
        # set number of nodes in each input, hidden, output layer
        self.inodes = inputnodes
        self.hnodes = hiddennodes
        self.onodes = outputnodes
    
    
        # link weight matrices(Gewichtsmatrix-bleibt erhalten) ->  wih (weight input) and who (weight output)
        # weights inside the arrays are w_i_j, where link is from node i to node j in the next layer
        # w11 w21
        # w12 w22 ...
        # numpy.random.normal() erhaelt 
        # - Mittelwert der Normalverteilung 0.0 
        # - die Standardabweichung bezogen auf die Anzahl der Knoten in der naechsten Schicht -> Anzahl der Knoten ^(-0.5)
        # - Groesse eines numpy-Arrays 
        self.wih = numpy.random.normal(0.0, pow(self.hnodes, - 0.5), (self.hnodes, self.inodes))  
        self.who = numpy.random.normal(0.0, pow(self.onodes, - 0.5), (self.onodes, self.hnodes))
    
    
    
        # learning rate
        self.lr = learningrate
        
        
        
        # Sigmoidfunktion um Ausgangssignal der versteckten Schicht zu erhalten
        # outputhidden = sigmoid(hiddenSchicht)
        # activation function is the sigmoid function
        self.activation_function = lambda x: scipy.special.expit(x)  # lambda als Hilfsfunktion, bekommt x und gibt an expit() weiter
        
        pass
    
    
    
    
    
    

    
    
    # train the neural network 
    # insgesamt 2 Trainingsphasen
    # input _list -> zu tranierende Input(Trainingsbeispiele)     taregts_list -> Zielantworten/Referenzen
    def train(self, inputs_list, targets_list):
        
    # Phase 1 berechnet die Ausgabe
        
        # Eingangsliste in 2d array konvertieren
        inputs = numpy.array(inputs_list, ndmin=2).T
        targets = numpy.array(targets_list, ndmin=2).T
        
        # Versteckte Schicht
        # hiddenSchicht = Verknuepfungsgewichte * Eingaenge          => werden zusammengefasst
        # calculate signals into hidden layer
        hidden_inputs = numpy.dot(self.wih, inputs)  # Punktprodukt
        
        # zusammengefassteUndmoderierte Signale von den Knoten der versteckten Schicht
        # calculate signals emerging from hidden layer
        hidden_outputs = self.activation_function(hidden_inputs)
        
        
        # Ausgabeschicht
    
        # hiddenSchicht = Verknuepfungsgewichte * versteckte Output   => werden zusammengefasst
        # calculate signals into final output layer
        final_inputs = numpy.dot(self.who, hidden_outputs)  # Punktprodukt
        
        # zusammengefassteUndmoderierte Signale von den Knoten der output Schicht
        # calculate signals emerging from final output layer
        final_outputs = self.activation_function(final_inputs)
    
        
        
        
    # Phase 2 Backpropagierung ( Fehlerinformationen gehen rueckwaerts um die Verknuepfungsgewichte genauer zu machen)
        # Verbesserung der Gewichte -> Fehler zwischen berechneter Ausgabe und Zielausgabe/Referenz
        
        
        # Fehler zwischen versteckter Schicht UND Ausgabeschicht 
        # Diff zwischen Referenz und Trainingsbeispiel
        output_errors = targets - final_outputs
        
        # Fehler zwischen Eingabeschicht UND versteckte Schicht
        # Fehler (output_errors) wird auf Verbindungsgewichte geteilt UND jeder Knoten der versteckten Schicht zusammengefasst
        # hidden layer error is the output_errors, split by weights, recombined at hidden nodes
        hidden_errors = numpy.dot(self.who.T, output_errors)
        
        
        
        # Gewichtskorrektur -> Verknuepfung zweier Knoten in aufeinanderfolgenden Schichten
        
        # Gewichtskorrektur zwischen versteckter Schicht UND Ausgabeschicht
        # update the weights for the links between the hidden and output layers
        # Gewichtskorrektur = Lernfaktor * Fehlerkomponente der naechsten Schicht * Sigmoidkomponente der naechsten Schicht * Transponierte Matrix der Ausgaenge von der vorherigen Schicht
        self.who += self.lr * numpy.dot((output_errors * final_outputs * (1.0 - final_outputs)), numpy.transpose(hidden_outputs))
        
        # Gewichtskorrektur zwischen versteckter Schicht UND Eingabeschicht 
        # update the weights for the links between the input and hidden layers
        self.wih += self.lr * numpy.dot((hidden_errors * hidden_outputs * (1.0 - hidden_outputs)), numpy.transpose(inputs))
        
        pass  
        
        
      
        
        
        
    # query the neural network
    # die Eingabesignale der Knoten der Eingabeschicht werden ueber die versteckte Schicht zur Ausgabeschicht geleitet
    def query(self, inputs_list): 
        # Eingangsliste in 2d array konvertieren
        inputs = numpy.array(inputs_list, ndmin=2).T
        
        # Versteckte Schicht
    
        # hiddenSchicht = Verknuepfungsgewichte * Eingaenge          => werden zusammengefasst
        # calculate signals into hidden layer
        hidden_inputs = numpy.dot(self.wih, inputs)  # Punktprodukt
        
        # zusammengefassteUndmoderierte Signale von den Knoten der versteckten Schicht
        # calculate signals emerging from hidden layer
        hidden_outputs = self.activation_function(hidden_inputs)
        
        
        # Ausgabeschicht
    
        # hiddenSchicht = Verknuepfungsgewichte * versteckte Output   => werden zusammengefasst
        # calculate signals into final output layer
        final_inputs = numpy.dot(self.who, hidden_outputs)  # Punktprodukt
        
        # zusammengefassteUndmoderierte Signale von den Knoten der output Schicht
        # calculate signals emerging from final output layer
        final_outputs = self.activation_function(final_inputs)
    
        return final_outputs










############ neural network mit 784 Knoten + Lernrate 0.3

############# NEURONALES NETZWERK DEFINIEREN

# number of input, hidden and output nodes
input_nodes = 784          #  Anzahl der Pixel der Bilder   -> 28*28=784
hidden_nodes = 100         #
output_nodes = 10          #  Anzahl der zu erkennenden Schilder

# learning rate is 0.3
learning_rate = 0.3

#create instance of neural network
neuralNet = neuralNetwork(input_nodes, hidden_nodes, output_nodes, learning_rate)


############# DATENSATZ ZUM TRAINIEREN LADEN 
# oeffne Datei
# Pfad   -r=readonly
#ref_data_file = open("Handschrift_mnist_dataset/mnist_train_100.csv", 'r')
ref_data_file = open("/home/pses/catkin_ws/src/echtzeitsysteme/src/NeuronalNetwork/Handschrift_mnist_dataset/mnist_train_100.csv", 'r')

# stellt die Datei zur verfuegung und liest sie ein (die Gesamte Datei in den Hauptspeicher - optimieren jeweils nur eine Zeile)
ref_data_list = ref_data_file.readlines()

# laenge  der Liste
len(ref_data_list)  # Debug Ausgabe

# schliesst den Input Stream aus der Datei
ref_data_file.close()




############# Neuronales Netzwerk mit zuvor geladener Datenliste trainieren
# durch Datenliste iterieren
for aufnehmen in ref_data_list:
    # String aufteilen in Zeilen bei dem Zeichen ','
    aufnehmen_all_values = aufnehmen.split(',')

    # Eingabeschicht Farbwerte(0...255) in (0.01 ... 1.0) bringen
    scaled_input = (numpy.asfarray(aufnehmen_all_values[1:]) / 255.0 * 0.99) + 0.01
    print(scaled_input) # Debug Ausgabe


    # Array erzeugen, welches nur 0len enthaelt, leange output_nodes hat (von 0...output_nodes-1)
    # +0.01 um gesaettigten Netzwerk zu verhindern da 0 und 1 nie erreichbar
    targets = numpy.zeros(output_nodes) + 0.01

    # konvergiert erste Element von String->int       
    # erste Element ist das soll Ergebnis des Outputs => wird auf 0.99 gesetzt
    targets[int(aufnehmen_all_values[0])] = 0.99

    # Aufruf zum trainieren mit scalierten Inputwerten(0.01,1.0) und targets(Zielantwort)
    neuralNet.train(scaled_input, targets)

    pass




############ neuronales Netzwerk anwenden (erst NACH dem Trainieren!!!)

############# DATENSATZ ZUM ANWENDEN LADEN 
# oeffne Datei
# Pfad   -r=readonly
anwenden_data_file = open("Handschrift_mnist_dataset/mnist_test_10.csv", 'r')

# stellt die Datei zur verfuegung und liest sie ein (die Gesamte Datei in den Hauptspeicher - optimieren jeweils nur eine Zeile)
anwenden_data_list = anwenden_data_file.readlines()

# laenge  der Liste
len(anwenden_data_list)  # Debug Ausgabe

# schliesst den Input Stream aus der Datei
anwenden_data_file.close()




######## Input einzeln testen

# String aufteilen in Zeilen bei dem Zeichen ','
anwenden_all_values = anwenden_data_list[0].split(',')

# gibt das den Referenz Wert des Bildes aus
print(anwenden_all_values[0])

#### Eingabe visuell dartstellen
# [1:] alles verwenden ausser das erste Element jeder Zeile (1 Element ist die Kennung !!!)
# asfarray() konvertiert Textstring in Zahlen in ein Array
# reshape() Matrix 28*28 Array als resultierendes Bild
output_image_array = numpy.asfarray(anwenden_all_values[1:]).reshape((28,28))

# Bild ausgeben mit Graustufen
matplotlib.pyplot.imshow(output_image_array, cmap='Greys', interpolation = 'None')


##### Ergebnis der Neuronalen Netzwerks
# Eingabeschicht Farbwerte(0...255) in (0.01 ... 1.0) bringen
anwenden_scaled_input = (numpy.asfarray(anwenden_all_values[1:]) / 255.0 * 0.99) + 0.01
# liefert Array mit Trefferquote zurueck (0.01,1.0)
neural_output = neuralNet.query(anwenden_scaled_input)
# liefert den maximalen Wert der Trefferquote zurueck
ergebnis = numpy.argmax(neural_output)
print(neural_output, "Neuronales Netzwerk Output")          # Debug
print(ergebnis, "Neuronales Netzwerk Ergebnis")             # Debug



