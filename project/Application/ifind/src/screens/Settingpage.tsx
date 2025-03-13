import React from "react";
import { StyleSheet, Text, TouchableOpacity, View } from "react-native";
import { SafeAreaProvider } from "react-native-safe-area-context";
import theme from "../styles/theme";

export const SettingPage = () => {
    const onItemPress = (item: string) => {
    console.log(`${item} 클릭됨`);
    };

return (
    <SafeAreaProvider>
        <View style={styles.container}>
        <Text style={styles.title}>설정</Text>
        <View style={styles.separator} />
        {/* 리스트 항목 */}
        <TouchableOpacity
            style={styles.listItem}
            onPress={() => onItemPress("알림 설정")}
        >
            <Text style={styles.listItemText}>알림 설정</Text>
        </TouchableOpacity>
        <TouchableOpacity
            style={styles.listItem}
            onPress={() => onItemPress("계정 관리")}
        >
            <Text style={styles.listItemText}>계정 관리</Text>
        </TouchableOpacity>
        <TouchableOpacity
            style={styles.listItem}
            onPress={() => onItemPress("고객센터")}
        >
            <Text style={styles.listItemText}>고객센터</Text>
        </TouchableOpacity>
        </View>
    </SafeAreaProvider>
);
};

const styles = StyleSheet.create({
container: {
    flex: 1,
    alignItems: "center",
    backgroundColor: "#fff",
},
title: {
    fontFamily: theme.mainfont,
    paddingTop: 10,
    margin: 10,
    fontSize: 35,
    // fontWeight: "bold",
    color: theme.mainDarkGrey,
},
separator: {
    width: "90%",
    height: 1,
    backgroundColor: theme.navicolor,
    marginBottom: "50%",
    },
listItem: {
    width: "90%",
    padding: 10,
    marginBottom: 20,
    borderBottomWidth: 0.5,
    borderColor: 'gray',
},
listItemText: {
    fontSize: 23,
    fontFamily: theme.mainfont,
    color: theme.mainDarkGrey,
},
});
