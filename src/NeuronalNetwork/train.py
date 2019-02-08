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
ref_data_file = open("Handschrift_mnist_dataset/mnist_train_100.csv", 'r')

# stellt die Datei zur verfuegung und liest sie ein (die Gesamte Datei in den Hauptspeicher - optimieren jeweils nur eine Zeile)
ref_data_list = ref_data_file.readlines()

# laenge  der Liste
len(ref_data_list)  # Debug Ausgabe

# schliesst den Input Stream aus der Datei
ref_data_file.close()

