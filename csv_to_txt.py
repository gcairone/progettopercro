import csv

# File CSV di input (contiene sec, nsec, x, y, z, qx, qy, qz, qw)
input_csv = 'bagfiles/gt-nc-quad-easy.csv'

# File di output (format .txt)
output_txt = 'bagfiles/gt-nc-quad-easy.txt'

# Apri il file CSV in lettura
with open(input_csv, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    
    # Salta l'intestazione del CSV se presente
    next(csv_reader, None)
    
    # Apri il file TXT in scrittura
    with open(output_txt, 'w') as txt_file:
        for row in csv_reader:
            # Estrai i valori di x, y, z, qx, qy, qz, qw
            sec = row[0]
            nsec = row[1]
            x = row[2]
            y = row[3]
            z = row[4]
            qx = row[5]
            qy = row[6]
            qz = row[7]
            qw = row[8]
            
            # Combina il sec e nsec per ottenere un timestamp completo (in secondi)
            timestamp = float(sec) + float(nsec) / 1e9
            
            # Scrivi nel file di output in formato txt
            txt_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
print(f"File convertito in {output_txt}")
