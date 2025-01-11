#include "key_value.h"



// 读取文件中的全部键值对，支持标题
unordered_map<string, unordered_map<string, string>> readKeyValuePairs(const string& filename)
{
    unordered_map<string, unordered_map<string, string>> all_kv_map;
    ifstream infile(filename);

    if (!infile.is_open()) {
        cerr << "无法打开文件进行读取！" << endl;
        return all_kv_map;
    }

    string line;
    string current_group;

    while (getline(infile, line)) {
        // 忽略空行或注释
        if (line.empty() || line[0] == '#') continue;

        // 如果是标题行，提取标题
        if (line[0] == '[' && line.back() == ']') {
            current_group = line.substr(1, line.size() - 2);  // 提取 [Group] 内的内容
            continue;
        }

        // 如果是键值对行
        size_t pos = line.find('=');
        if (pos != string::npos && !current_group.empty()) {
            string key = line.substr(0, pos);
            string value = line.substr(pos + 1);

            // 存入标题下的键值对
            all_kv_map[current_group][key] = value;
        }
    }

    infile.close();
    return all_kv_map;
}

// 将键值对全部写入文件，带标题
void writeKeyValuePairs(const string& filename, const unordered_map<string, unordered_map<string, string>>& all_kv_map)
{
    ofstream outfile(filename);

    if (!outfile.is_open()) {
        cerr << "无法打开文件进行写入！" << endl;
        return;
    }

    // 将每个组的键值对写入文件
    for (const auto& group : all_kv_map) {
        // 写入组标题
        outfile << "[" << group.first << "]" << endl;

        // 写入组内的键值对
        for (const auto& pair : group.second) {
            outfile << pair.first << "=" << pair.second << endl;
        }

        outfile << endl;  // 每个组之间留空行
    }

    outfile.close();
}

// 修改指定标题和键的值
void modifyKeyValue(const string& filename, const string& title, const string& key, const string& value) {
    // 读取文件中的键值对
    unordered_map<string, unordered_map<string, string>> all_kv_map = readKeyValuePairs(filename);

    // 如果文件读取成功，且找到指定标题
    if (!all_kv_map.empty()) {

        printf(" 当前的键值对: \n");

        for (const auto& group : all_kv_map) {
            printf("\n[%s]\n", group.first.c_str());  // 输出标题，group.first 是 string 类型，需要使用 c_str() 转为 C 风格字符串
            for (const auto& pair : group.second) {
                printf("%s=%s\n", pair.first.c_str(), pair.second.c_str());  // 输出键值对，pair.first 和 pair.second 都是 string 类型
            }
        }

        // 检查是否有对应的标题
        auto group = all_kv_map.find(title);
        if (group != all_kv_map.end()) {
            // 修改指定标题下的键值对
            group->second[key] = value;

            // 将修改后的键值对写回文件
            writeKeyValuePairs(filename, all_kv_map);

            printf("\n\n 键值对修改完成 \n\n");

        } else {
            cerr << "未找到标题: " << title << endl;
        }
    } else {
        cerr << "文件为空或读取失败！" << endl;
    }
}

// 读取指定标题和键的键值对
string getKeyValue(const string& filename, const string& title, const string& key) {
    // 读取文件中的键值对
    unordered_map<string, unordered_map<string, string>> all_kv_map = readKeyValuePairs(filename);

    // 查找指定标题和键
    auto group = all_kv_map.find(title);
    if (group != all_kv_map.end()) {
        auto kv = group->second.find(key);
        if (kv != group->second.end()) {
            return kv->second;  // 返回键值
        } else {
            cerr << "未找到键: " << key << " 在标题: " << title << endl;
        }
    } else {
        cerr << "未找到标题: " << title << endl;
    }

    return "";  // 如果没有找到，返回空字符串
}

// 写入指定标题和键的键值对
void setKeyValue(const string& filename, const string& title, const string& key, const string& value) {
    // 读取文件中的键值对
    unordered_map<string, unordered_map<string, string>> all_kv_map = readKeyValuePairs(filename);

    // 检查是否存在指定标题
    auto group = all_kv_map.find(title);
    if (group != all_kv_map.end()) {
        // 如果标题存在，修改或添加指定的键值对
        group->second[key] = value;
    } else {
        // 如果标题不存在，添加一个新的标题并插入键值对
        all_kv_map[title][key] = value;
    }

    // 将修改后的键值对写回文件
    writeKeyValuePairs(filename, all_kv_map);
}

// int main() {
//     const string filename = "data.txt";

//     // 测试修改键值对
//     modifyKeyValue(filename, "Group1", "key1", "66666666666");

//     // 测试读取指定键值对
//     string value = getKeyValue(filename, "Group1", "key1");

//     cout << "读取的键值对: key1 = " << value << endl;

//     // 测试写入指定键值对
//     setKeyValue(filename, "Group2", "key5", "12345");

//     cout << "修改后的文件内容: \n";
    
//     system("cat ./data.txt");

//     return 0;
// }
