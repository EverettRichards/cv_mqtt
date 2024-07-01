import chardet

with open('/home/udel/EdgeCode/tf_models/ssd-labels.txt', 'r', encoding='None') as f:
    lines = f.readlines()

# Write the file with UTF-8 encoding
with open('/home/udel/EdgeCode/tf_models/ssd-labels-utf8.txt', 'w', encoding='utf-8') as f:
    f.writelines(lines)