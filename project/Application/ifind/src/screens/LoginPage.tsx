import AsyncStorage from '@react-native-async-storage/async-storage';
import { NativeStackNavigationProp } from '@react-navigation/native-stack';
import React, { useState } from 'react';
import { Alert, Image, StatusBar, StyleSheet, Text, TextInput, TouchableOpacity, View } from 'react-native';
import { signIn } from '../lib/auth';
import theme from '../styles/theme';

interface LoginPageProps {
    navigation: NativeStackNavigationProp<any, 'LoginPage'>;
}

export const LoginPage = ({navigation}: LoginPageProps) =>  {

    const [form, setForm] = useState<{ email: string; password: string }>({
        email: "",
        password: "",
    });
    const handleInputChange = (field: keyof typeof form, value: string) => {
        setForm({
            ...form,
            [field]: value,
        });
    };
    const signInSubmit = async () => {
        const {email, password} = form;
        try {
            const {user} = await signIn({email, password});
            console.log(user);
            await AsyncStorage.setItem('userUID', user.uid);
            navigation.navigate('MainPage', {name: 'MainPage'})
        } catch (e: any) {
            // const errorCode = e.code as string;
            const alertMessage = "아이디 또는 비밀번호를 확인해주세요!";
            Alert.alert("로그인 실패", alertMessage);
        }
    };

    const goToSignupPage = () => {
        navigation.navigate('SignupPage', {name: 'SignupPage'});
    };

    return (
    <View style={styles.container}>
    <Image source={require("../asset/mainicon.png")} style={styles.logo_image}/>
    <Text style={styles.title}>어서오세요!</Text>
    <StatusBar hidden={true} />

    <View style={styles.inputContainer}>
        <TextInput style={styles.info} placeholder="이메일"
        placeholderTextColor='#3e3e3e'
        value={form.email}
        onChangeText={(value) => handleInputChange('email', value)}/>
    </View>
    <View style={styles.inputContainer}>
        <TextInput style={styles.info} placeholder="비밀번호" secureTextEntry={true}
        placeholderTextColor='#3e3e3e'
        value={form.password}
        onChangeText={(value) => handleInputChange('password', value)}/>
    </View>

    <TouchableOpacity
        style={styles.login_button}
        onPress={signInSubmit}
    >
        <Text style={styles.buttonText}>로그인하기</Text>
    </TouchableOpacity>

    <TouchableOpacity
        onPress={goToSignupPage}
        >
        <Text style={styles.gotosignuptext}>회원이 아니라면? 회원가입</Text>
        </TouchableOpacity>
    </View>
);
}

const styles = StyleSheet.create({
container: {
    flex: 1,
    backgroundColor: '#fff',
    alignItems: 'center',
    justifyContent: 'center',
},
title: {
    fontSize: 45,
    // fontWeight: "bold",
    fontFamily: theme.mainfont,
    marginBottom: 50,
    marginTop: 10,
    color: theme.mainDarkGrey,
},
logo_image: {
    width: 100,
    height: 100,
},
info: {
    fontSize: 22,
    fontFamily: theme.mainfont,
    color: theme.mainDarkGrey,
},
inputContainer: {
    alignItems: 'center',
    borderWidth: 3,
    borderColor: theme.signupandin,
    borderRadius: 40,
    marginBottom: 10,
    paddingHorizontal: 10,
    paddingVertical: 5,
    width: '70%',
    color: theme.mainDarkGrey,
},
login_button: {
    width: '40%',
    color: theme.mainDarkGrey,
    alignItems: 'center',
    marginTop: 20,
    backgroundColor: theme.mainOrange,
    paddingHorizontal: 20,
    paddingVertical: 10,
    borderRadius: 40,
    shadowColor: "#000",
        shadowOffset: {
            width: 0,
            height: 2,
        },
        shadowOpacity: 0.25,
        shadowRadius: 4,
},
buttonText: {
    padding: 5,
    color: 'black',
    fontSize: 23,
    fontFamily: theme.mainfont,
},
gotosignuptext: {
    marginTop: 5,
    color:'gray',
    fontSize: 20,
    textDecorationLine: 'underline',
}
});
